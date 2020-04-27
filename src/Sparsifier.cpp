/*
 * Sparsifier.cpp
 *
 *  Created on: Jan 29, 2018
 *      Author: Jerry Hsiung
 */

#include "Sparsifier.h"
#include "SparseOptimizer.h"
#include "LandmarkPoseFactor.h"
#include "PseudoPriorFactor.h"
#include "PseudoPriorFrontFactor.h"
#include "RelativePosePrior.h"
#include "gtsam/linear/HessianFactor.h"
#include "gtsam/linear/RegularHessianFactor.h"
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/PriorFactor.h>

#if 0
std::ostream &operator<<(std::ostream &s, const g2o::HyperGraph::Vertex *v)
{
    return s << (void *) v << "[" << v->id() << "]";
}

std::ostream &operator<<(std::ostream &s, const g2o::HyperGraph::Edge *e)
{
    return s << e->vertices();
}
#endif
Sparsifier::Sparsifier() : _graph(NonlinearFactorGraph()), _mbgraph(NonlinearFactorGraph())
{
    // LinearSolver *linearSolver = new LinearSolver;
    // linearSolver->setBlockOrdering(false);
    // BlockSolver *blockSolver = new BlockSolver(linearSolver);
    // OptimizationAlgorithm *solver = new OptimizationAlgorithm(blockSolver);
    // _subgraph->setAlgorithm(solver);
}

Sparsifier::~Sparsifier()
{
}

void Sparsifier::setGraph(const NonlinearFactorGraph graph, const Values values, const std::map<size_t, FactorStruct> factors) {
    _graph = graph;
    _values = values;
    _factors = factors;

    _mbgraph.resize(0);
    _mbValues.clear();
    _mbFactors.clear();

    _marginalizeKey.clear();
    _landmarkKeys.clear();
    _priorKeys.clear();
    _topologySkeleton.clear();
    _parameterMapping.clear();
    _sparseMapping.clear();
    _sparseInfos.clear();
    _sparsifiedGraph.resize(0);
    _spFactors.clear();
}

void Sparsifier::setNextStates(KeyVector ns)
{
   _nextStatesKeys = ns;
   for (Key key: _nextStatesKeys)
      std::cout << " Next state is " << DefaultKeyFormatter(key) << std::endl;
}

void Sparsifier::setTargetInfo(const Eigen::MatrixXd& targetInfo) {
  _targetInfo = 0.5*(targetInfo+targetInfo.transpose()).eval();

  // const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

  // fstream trajfile;
  // trajfile.open ("/home/shsiung/sparsification/target_info_orig"+std::to_string(file_idx_)+".csv", std::ofstream::out | std::ofstream::app);
  // if (trajfile.is_open()) {  
  //    trajfile.setf(ios_base::fixed);
  //    trajfile <<  _targetInfo.format(CSVFormat);
  //    trajfile.close();
  // }
  // file_idx_++;

  std::cout << " Target Info size: " << _targetInfo.rows() << " , " << _targetInfo.cols() <<
  std::endl;
}

void Sparsifier::calculateMarkovBlanket(const KeyVector& marginalizableKeys) {
  _marginalizeKey = marginalizableKeys;
  std::set<size_t> removedFactorSlots;
  const VariableIndex variableIndex(_graph);
  for(Key key: marginalizableKeys) {
    const FastVector<size_t>& slots = variableIndex[key];
    removedFactorSlots.insert(slots.begin(), slots.end());
  }

  // Calculate markov blanket
  for(size_t slot: removedFactorSlots) {
    gtsam::FactorGraph<gtsam::NonlinearFactor>::sharedFactor fac = _graph.at(slot); 
    if (fac) {
      _mbgraph.push_back(_graph.at(slot));
      _mbFactors.insert(std::pair<size_t, FactorStruct>(slot,_factors.at(slot)));
      for (Key key: fac->keys())
      {
        // key != _marginalizeKey[0] is to remove marginalized keys in book keeping
        if (_factors.at(slot).factorType_ == FactorType::Projection && key != _marginalizeKey[0])
        {
          //std::cout << _factors.at(slot).factorType_ << " Insert landmark key " << DefaultKeyFormatter(key) << std::endl;
          if (std::find(_landmarkKeys.begin(), _landmarkKeys.end(),key)==_landmarkKeys.end())
          {
            _landmarkKeys.push_back(key);
          }
        }
        else if (_factors.at(slot).factorType_ == FactorType::Prior)// && fac->keys().size() == 7)
        {
          // SLOW since check entire list every time
          if (std::find(_priorKeys.begin(), _priorKeys.end(),key)==_priorKeys.end() && key != marginalizableKeys[0])
          {
            //std::cout << _factors.at(slot).factorType_ <<  " Insert prior key " << DefaultKeyFormatter(key) << std::endl;
            _priorKeys.push_back(key);
          }
        }
        // If exist, then insert, otherwise don't insert
       //std::cout << " Inserting value at " << DefaultKeyFormatter(key) << std::endl;
         _mbValues.tryInsert(key, _values.at(key));
      }
    }
  }


}

void Sparsifier::buildJacobianMapping()
{
    // //g2o::JacobianWorkspace jw;

    // Set index mapping
    size_t k = 0;
    for(Key key: _mbgraph.keys())
    {
        if (key != _marginalizeKey[0])
        {
          _parameterMapping.insert(std::pair<Key,size_t>(key, k));
          // std::cout << " key involved " << DefaultKeyFormatter(key) 
          //           << "(" << _mbValues.at(key).dim() << ")" 
          //           << " at index " << k << "-" << k+_mbValues.at(key).dim()-1 << std::endl;
          k +=_mbValues.at(key).dim();
        }
    }
    
    Pose3 marg_pose = _mbValues.at<Pose3>(_marginalizeKey[0]);

    for (const FactorStruct& fs: _topologySkeleton)
    {
        // Pose3-Point3 factor
        if (fs.factorType_ == FactorType::SP_lmP)
        {
            _sparseMapping.push_back(MeasurementJacobian()); // Add a measurement jacobian
            Key poseKey = fs.keys_[0];
            Key lmKey = fs.keys_[1];
            Pose3 pose = _mbValues.at<Pose3>(poseKey);
            Point3 lm = _mbValues.at<Point3>(lmKey);
            Point3 BApt = pose.translation() - pose.transform_pose_to(Pose3(Rot3(),Point3(lm))).translation();
            gtsam::Matrix Jp, Jlm;

            // std::cout << " insert landmark " << DefaultKeyFormatter(lmKey) << std::endl;

            LandmarkPoseFactor lmp(BApt);
            lmp.evaluateError(pose,lm,Jp,Jlm);
            _sparseMapping.back().push_back(std::make_pair(Jp, _parameterMapping[poseKey]));
            _sparseMapping.back().push_back(std::make_pair(Jlm, _parameterMapping[lmKey]));


        }
        else if (fs.factorType_ == FactorType::SP_lmPrior)
        {
             gtsam::Matrix Jprior = gtsam::Matrix::Identity( 3 , 3 );
            _sparseMapping.push_back(MeasurementJacobian()); // Add a measurement jacobian
            _sparseMapping.back().push_back(std::make_pair(Jprior, _parameterMapping[fs.keys_[0]]));
        }
        else if (fs.factorType_ == FactorType::SP_P) // Dense prior
        {
            // size_t prior_dim = 0;
            // size_t startIdx = 9999;
            // for (Key prior_key: fs.keys_)
            // {
            //   if (_parameterMapping[prior_key] < startIdx)
            //   {
            //     startIdx = _parameterMapping[prior_key];
            //   }
            //   prior_dim += _mbValues.at(prior_key).dim();
            // }
            // // std::cout << " Adding prior key " << 
            // //         DefaultKeyFormatter(fs.keys_[0]) << 
            // //          " dim " <<  prior_dim << " starting at " << startIdx <<  std::endl;
            
            // gtsam::Matrix Jp = Matrix::Identity(15,30);
            // Jp.block<6,6>(0,0) = Matrix::Identity(6,6);
            // Jp.block<3,3>(6,6) = Matrix::Identity(6,6);
            // Jp.block<6,6>(9,9) = Matrix::Identity(6,6);
            // Jp.block<6,6>(0,15) = -Matrix::Identity(6,6);
            // Jp.block<3,3>(6,21) = -Matrix::Identity(6,6);
            // Jp.block<6,6>(9,24) = -Matrix::Identity(6,6);

            // _sparseMapping.push_back(MeasurementJacobian()); // Add a measurement jacobian
            // _sparseMapping.back().push_back(std::make_pair(Jp, startIdx));

             size_t prior_dim = 0;
            for (Key prior_key: fs.keys_)
            {
                prior_dim += _mbValues.at(prior_key).dim();
            }
            // std::cout << " Adding prior key " << 
            //         DefaultKeyFormatter(fs.keys_[0]) << 
            //          " dim " <<  prior_dim << " starting at " << _parameterMapping[fs.keys_[0]] <<  std::endl;
            gtsam::Matrix Jp = gtsam::Matrix::Identity( prior_dim , prior_dim );
            _sparseMapping.push_back(MeasurementJacobian()); // Add a measurement jacobian
            _sparseMapping.back().push_back(std::make_pair(Jp, _parameterMapping[fs.keys_[0]]));
        }
    }

    // // For each edge, call "evaluationError" function on the current linearization point
    // // Put into the JacobianMapping indices corresponding to the targetInformation
    // // TODO:
    // //   What is the ordering of targetInfo?


    // TODO:
    //   Local or global linearization point? How to?
}

void Sparsifier::sparsifyTopology()
{
    size_t idx = 0;
    map<Key,bool> landmarkIncluded; // Prevent the landmark measurement included twice
    for (const auto& fs: _mbFactors)
    {
       // std::cout << " Factor type " << static_cast<size_t>(fs.second.factorType_) << std::endl;
       // for (Key key: fs.second.keys_)
       //      {
       //        std::cout << " --- key " << DefaultKeyFormatter(key) << std::endl;
       //      }
        bool skip = false;
        if(fs.second.factorType_ == FactorType::Projection)
        {  
           KeyVector factorKeys;
           KeyVector lmKey;
           for (Key key: fs.second.keys_)
            {
              if (key == _marginalizeKey[0])
                //std::cout << _factors.at(slot).factorType_ << " Insert landmark key " << DefaultKeyFormatter(key) << std::endl;
                factorKeys.push_back(_nextStatesKeys[0]);
              else
              {
                if (landmarkIncluded.find(key) == landmarkIncluded.end())
                {
                  landmarkIncluded.insert(std::pair<Key,bool>(key, true));
                  factorKeys.push_back(key);
                  lmKey.push_back(key);

                }
                else
                {
                  skip = true;
                }
              }
            }
            if(!skip)
            {
              _topologySkeleton.push_back(FactorStruct(FactorType::SP_lmP, idx, factorKeys));
              idx++;
              //_topologySkeleton.push_back(FactorStruct(FactorType::SP_lmPrior, idx, lmKey));
              //idx++;

            }
        }
    }
    //  std::cout << " Adding prior type at idx" << idx<< std::endl;
    _topologySkeleton.push_back(FactorStruct(FactorType::SP_P, idx, _priorKeys));
}

void Sparsifier::optimizeSparseInformation()
{
  // Call SparseOptimizer on TargetInfo and JacobianMapping
  _sparseInfos = optimizeInformation(_sparseMapping, _targetInfo);

  //for (auto info: _sparseInfos)
  //   std::cout << info << std::endl<<std::endl;
  // std::cout << " ===================== " << std::endl;
  // std::cout << _targetInfo << std::endl;
}

NonlinearFactorGraph Sparsifier::updateMarginalGraph()
{
  // Compute KLD? 
  // Check consistensy? Check Chi2()?
  // Return a NonlinearFactorGraph such that the edges are the newly created edges with
  //   the correct information calculated by optimizeInformation()

  Pose3 marg_pose = _mbValues.at<Pose3>(_marginalizeKey[0]);
  std::cout << " Sparse Info Size " << _sparseInfos.size() << std::endl;

  for (const FactorStruct& fs: _topologySkeleton)
  {
      // Pose3-Point3 factor
      if (fs.factorType_ == FactorType::SP_lmP)
      {
        _sparseMapping.push_back(MeasurementJacobian()); // Add a measurement jacobian
        Key poseKey = fs.keys_[0];
        Key lmKey = fs.keys_[1];
        Pose3 pose = _mbValues.at<Pose3>(poseKey);
        Point3 lm = _mbValues.at<Point3>(lmKey);
        // Measurement
        Point3 BApt = pose.translation() - pose.transform_pose_to(Pose3(Rot3(),Point3(lm))).translation();        
        // Adding sparsified noise model
        std::list<Eigen::MatrixXd>::iterator it = _sparseInfos.begin();
        std::advance(it, fs.index_);
        // std::cout << " Pose3Point3 info " << fs.index_ << std::endl;
        // std::cout << *it << std::endl;

        const noiseModel::Gaussian::shared_ptr mea_noise = noiseModel::Gaussian::Information(*it);
        //const noiseModel::Gaussian::shared_ptr mea_noise = noiseModel::Gaussian::Information(*it);

       //const noiseModel::Gaussian::shared_ptr mea_noise = noiseModel::Gaussian::Information(Matrix::Identity(3,3)*0.5);
        LandmarkPoseFactor lmp(poseKey, lmKey, BApt, mea_noise);
        //PriorFactor<Point3> lmp(lmKey, lm, mea_noise);
        _sparsifiedGraph.add(lmp);

       

      }
      else if (fs.factorType_ == FactorType::SP_lmPrior)
      {
         std::list<Eigen::MatrixXd>::iterator it = _sparseInfos.begin();
        std::advance(it, fs.index_);
        // _sparsifiedGraph.add(PriorFactor<Point3>(fs.keys_[0], _mbValues.at<Point3>(fs.keys_[0]), 
        //       noiseModel::Gaussian::Information(Matrix::Identity(3,3))));
        _sparsifiedGraph.add(PriorFactor<Point3>(fs.keys_[0], _mbValues.at<Point3>(fs.keys_[0]), 
              noiseModel::Gaussian::Information(*it)));
      }
      else if (fs.factorType_ == FactorType::SP_P)
      {
         std::list<Eigen::MatrixXd>::iterator it = _sparseInfos.begin();
         std::advance(it, fs.index_);

         size_t dim=0;
         //  std::cout << " Prior info " << fs.index_ << std::endl;
          // std::cout <<  *it << std::endl;

        for (Key key: fs.keys_)
        {
           dim += _mbValues.at(key).dim();
        }
        // PseudoPriorFrontFactor dense_prior(fs.keys_,
        //                               _mbValues.at<imuBias::ConstantBias>(fs.keys_[0]),
        //                               _mbValues.at<Vector3>(fs.keys_[1]),
        //                               _mbValues.at<Pose3>(fs.keys_[2]),
        //                               *it,dim);
        // _sparsifiedGraph.add(dense_prior);

         Matrix info = *it;
        _sparsifiedGraph.add(PriorFactor<imuBias::ConstantBias>(fs.keys_[0], _mbValues.at<imuBias::ConstantBias>(fs.keys_[0]), 
              noiseModel::Gaussian::Information(info.block<6,6>(0,0))));
        _sparsifiedGraph.add(PriorFactor<Vector3>(fs.keys_[1], _mbValues.at<Vector3>(fs.keys_[1]), 
              noiseModel::Gaussian::Information(info.block<3,3>(6,6))));
        _sparsifiedGraph.add(PriorFactor<Pose3>(fs.keys_[2], _mbValues.at<Pose3>(fs.keys_[2]), 
              noiseModel::Gaussian::Information(info.block<6,6>(9,9))));
       
        // _sparsifiedGraph.add(PriorFactor<imuBias::ConstantBias>(fs.keys_[0], _mbValues.at<imuBias::ConstantBias>(fs.keys_[0]), 
        //       Matrix::Identity(6,6)));
        // _sparsifiedGraph.add(PriorFactor<Vector3>(fs.keys_[1], _mbValues.at<Vector3>(fs.keys_[1]), 
        //       Matrix::Identity(3,3)));
        // _sparsifiedGraph.add(PriorFactor<Pose3>(fs.keys_[2], _mbValues.at<Pose3>(fs.keys_[2]), 
        //       Matrix::Identity(6,6)));

        // PseudoPriorFrontFactor dense_prior(fs.keys_,
        //                               _mbValues.at<imuBias::ConstantBias>(fs.keys_[0]),
        //                               _mbValues.at<Vector3>(fs.keys_[1]),
        //                               _mbValues.at<Pose3>(fs.keys_[2]),
        //                               *it*1e-5,dim);
        // _sparsifiedGraph.add(dense_prior);
        // imuBias::ConstantBias bias1 = _mbValues.at<imuBias::ConstantBias>(fs.keys_[0]);
        // Vector vel1 = _mbValues.at<Vector3>(fs.keys_[2]);
        // Pose3 x1 = _mbValues.at<Pose3>(fs.keys_[4]);
        // imuBias::ConstantBias bias2 = _mbValues.at<imuBias::ConstantBias>(fs.keys_[1]);
        // Vector vel2 = _mbValues.at<Vector3>(fs.keys_[3]);
        // Pose3 x2 = _mbValues.at<Pose3>(fs.keys_[5]);

        // Vector6 x_rel = x2.localCoordinates(x1);
        // Vector3 vel_rel = vel2 - vel1;
        // Vector6 bias_rel = bias2.vector() - bias1.vector();

        // RelativePosePrior relPrior(fs.keys_,bias_rel,vel_rel,x_rel, (*it), dim);
        // _sparsifiedGraph.add(relPrior);

        // // ***** LOGGING *******
                  //  const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

                  // fstream trajfile;
                  // trajfile.open ("/home/shsiung/sparsification/sparse_info"+std::to_string(file_idx_)+".csv", std::ofstream::out | std::ofstream::app);
                  // Eigen::MatrixXd sparse_out = Eigen::MatrixXd::Zero(15+3*_landmarkKeys.size(),15+3*_landmarkKeys.size());
                  // size_t k = 0;
                  // for (auto& info :_sparseInfos)
                  // {
                  //   std::cout << " info : " << std::endl;
                  //   std::cout << info << std::endl;
                  //   if(info.rows()==3)
                  //     sparse_out.block<3,3>(k,k) = info;
                  //   else
                  //     sparse_out.block<15,15>(k,k) = info;
                  //   k += info.rows();
                  // }

                  // if (trajfile.is_open()) {  
                  //      trajfile.setf(ios_base::fixed);
                  //      trajfile <<  sparse_out.format(CSVFormat);
                  //      trajfile.close();
                  // }
                  // trajfile.open ("/home/shsiung/sparsification/target_info"+std::to_string(file_idx_)+".csv", std::ofstream::out | std::ofstream::app);
                  // if (trajfile.is_open()) {  
                  //      trajfile.setf(ios_base::fixed);
                  //      trajfile <<  _targetInfo.format(CSVFormat);
                  //      trajfile.close();
                  // }

                  // trajfile.open ("/home/shsiung/sparsification/H"+std::to_string(file_idx_)+".csv", std::ofstream::out | std::ofstream::app);
                  // Eigen::MatrixXd H = Eigen::MatrixXd::Zero(15+3*_landmarkKeys.size(),_targetInfo.rows());
                  // size_t t = 0;
                  // for (auto& jm: _sparseMapping)
                  // {
                  //   for (auto& jacobentry : jm)
                  //   {
                  //     if (jacobentry.first.rows() == 3)
                  //       H.block<3,3>(t,jacobentry.second) = jacobentry.first;
                  //     else
                  //       H.block<15,30>(t,jacobentry.second) = jacobentry.first;
                  //   }
                  //   t += jm.front().first.rows();
                  // }

                  // if (trajfile.is_open()) {  
                  //      trajfile.setf(ios_base::fixed);
                  //      trajfile <<  H.format(CSVFormat);
                  //      trajfile.close();
                  // }

                  // file_idx_++;
        // ***** LOGGING *******

      }
  }

  return _sparsifiedGraph;
}

void Sparsifier::test()
{
    // _mbgraph.print();
    std::cout << " subvalue size " << _mbValues.size() << std::endl;
    std::cout << " graph isze " << _mbgraph.size() << std::endl;
    std::cout << " factors size " << _mbFactors.size() << std::endl;
    std::cout << " Sparse Topology " << std::endl;
    // for (const FactorStruct& fs: _topologySkeleton)
    // {
    //     std::cout << " Factor " << fs.index_ << " : " << static_cast<int>(fs.factorType_) << " at " << 
    //     DefaultKeyFormatter( fs.keys_[0]) << " , " <<DefaultKeyFormatter( fs.keys_[1]) << std::endl;
    // }
    size_t jacsize=0;
    for(auto &jaclist: _sparseMapping) {
        jacsize += jaclist.front().first.rows();
    }
    std::cout << " Sparse Jacob size " << jacsize << std::endl;
    std::cout << " Target Jacob size " << _targetInfo.rows() << std::endl;
}