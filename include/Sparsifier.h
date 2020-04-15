/*
 * Sparsifier.h
 *
 *  Created on: Jan 29, 2018
 *      Author: Jerry Hsiung
 */
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <queue>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <gtsam/inference/Symbol.h>   
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <SparseOptimizer.h>
#include <Eigen/Dense>


#ifndef SPARSIFIER_H_
#define SPARSIFIER_H_

using namespace gtsam;
enum FactorType{Unknown = 0, Preint = 1, Projection = 2, 
                Prior = 3, Bias = 4, SP_lmP = 5, 
                SP_P = 6, Global = 7, Relative = 8,
                SP_lmPrior = 9};

struct FactorStruct{
  FactorStruct(FactorType ft, size_t idx, KeyVector keys):
  factorType_(ft), index_(idx), keys_(keys)
  {}

  FactorType factorType_;
  size_t index_;
  KeyVector keys_;
};


class Sparsifier {

public:
    typedef std::set<Key *,
            bool (*)(Key *,
                    Key *)> VertexSet;
    //typedef g2o::HyperGraph::EdgeSet EdgeSet;
    typedef std::map<Key *,
            Key *> VertexMapping;
    typedef std::set<size_t> FactorIndex;
    typedef std::vector<gtsam::FactorGraph<gtsam::NonlinearFactor>::sharedFactor>  FactorVector;

    Sparsifier();
    ~Sparsifier();

    // NOTE:: Don't need options, we have designed topology
    //void setSparsityOptions(const SparsityOptions &opts);
    void setGraph(const NonlinearFactorGraph graph, const Values values, 
                  const std::map<size_t, FactorStruct> factors);

    void setNextStates(KeyVector ns);

    void setTargetInfo(const Eigen::MatrixXd& targetInfo);

    // Return a set of new factors that we define according to our topology
    NonlinearFactorGraph remove(
            Key& marginalizableKey);
    NonlinearFactorGraph remove(
            const KeyVector& marginalizableKeys);

    // KeyVector extendedMarkovBlanketVertices(
    //         Key *root, const KeyVector &pickBin,
    //         KeyVector &picked) const;
    // KeyVector markovBlanketVertices(
    //         Key *root) const;
    // FactorVector markovBlanketEdges(
    //         Key *root) const;

    void calculateMarkovBlanket(const KeyVector& marginalizableKeys);

    void test();

    void sparsifyTopology();

    void buildJacobianMapping();

    void optimizeSparseInformation();

    NonlinearFactorGraph updateMarginalGraph();

private:
    FactorVector markovBlanketEdges(
            const KeyVector &mbVertices,
            const KeyVector &hubs) const;

    void addSubgraphVertex(
            int id, Key *original,
            const Key *reparam = NULL);

    void addSubgraphEdge(const NonlinearFactorGraph& edge);

    void buildSubgraph(
            const KeyVector &toRemove,
            const KeyVector &blanketVertices,
            const NonlinearFactorGraph &blanketEdges);

   // TopologyProvider *chooseTopologyProvider();

    void computeTargetInformation(
            const KeyVector &toRemove);

    void updateInputGraph(
            const KeyVector &toRemove,
            const NonlinearFactorGraph &blanketEdges,
            const NonlinearFactorGraph &newEdges,
            const std::list<Eigen::MatrixXd> &infomats);

    void cleanup();

    VertexSet mapForward(const VertexSet &vset);


private:
//    SparsityOptions _opts;
    NonlinearFactorGraph _graph;
    Values _values;
    std::map<size_t, FactorStruct> _factors;
    KeyVector _nextStatesKeys;

    // Markov Blanket Graph (including marginalized keys)
    NonlinearFactorGraph _mbgraph;
    Values _mbValues;
    Eigen::MatrixXd _targetInfo;
    std::map<size_t, FactorStruct> _mbFactors;  //index, FactorStruct

    KeyVector _marginalizeKey;
    KeyVector _landmarkKeys, _priorKeys;
    std::map<size_t, FactorStruct> _spFactors; 
    std::vector<FactorStruct> _topologySkeleton;
    std::map<Key, size_t> _parameterMapping;
    JacobianMapping _sparseMapping;
    std::list<Eigen::MatrixXd> _sparseInfos;
    NonlinearFactorGraph _sparsifiedGraph;

    size_t file_idx_ = 0;
};

#endif /* SPARSIFIER_H_ */