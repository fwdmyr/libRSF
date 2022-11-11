//
// Created by haoming on 11.11.22.
//

/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)
 ***************************************************************************/

/**
 * @file Test_IV19_GNSS.cpp
 * @author Tim Pfeifer and Leopold Mauersberger
 * @date 04 Mar 2021
 * @brief File containing a test for an application for 3D pose estimation based on pseudo range measurements. This special version is made for the Intelligent Vehicles Symposium 2019.
 * @copyright GNU Public License.
 *
 */

#include "IV19_GNSS.h"

/** define paths */
#define DATA_BPB "../../datasets/smartLoc/Berlin_Potsdamer_Platz"
#define DATA_BGM "../../datasets/smartLoc/Berlin_Gendarmenmarkt"
#define DATA_FMT "../../datasets/smartLoc/Frankfurt_Main_Tower"
#define DATA_FWT "../../datasets/smartLoc/Frankfurt_Westend_Tower"

#define ERROR_GAUSS "gauss"
#define ERROR_VBISM "stsm_vbi"
#define ERROR_VBIMM "stmm_vbi"

/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2018 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)
 ***************************************************************************/

/**
 * @file IV19_GNSS.cpp
 * @author Tim Pfeifer
 * @date 04 Jun 2019
 * @brief File containing an application for 3D pose estimation based on pseudo range measurements. This special version is made for the Intelligent Vehicles Symposium 2019.
 * @copyright GNU Public License.
 *
 */

#include "IV19_GNSS.h"

/** @brief Build the factor Graph with initial values and a first set of measurements
 *
 * @param Graph reference to the factor graph object
 * @param Measurements reference to the dataset that contains the pseudo range measurements
 * @param Config reference to the factor graph config
 * @param Options solver option to estimate initial values
 * @param TimestampFirst first timestamp in the Dataset
 * @return nothing
 *
 */
void InitGraph(libRSF::FactorGraph &Graph,
               libRSF::SensorDataSet &Measurements,
               libRSF::FactorGraphConfig const &Config,
               ceres::Solver::Options Options,
               double TimestampFirst)
{
  /** build simple graph */
  libRSF::FactorGraphConfig SimpleConfig = Config;
  SimpleConfig.GNSS.ErrorModel.Type = libRSF::ErrorModelType::Gaussian;
  libRSF::FactorGraph SimpleGraph;

  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point3, TimestampFirst);
  SimpleGraph.addState(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, TimestampFirst);
  AddPseudorangeMeasurements(SimpleGraph, Measurements, SimpleConfig, TimestampFirst);

  /** solve */
  Options.minimizer_progress_to_stdout = false;
  SimpleGraph.solve(Options);

  /**add first state variables */
  Graph.addState(POSITION_STATE, libRSF::DataType::Point3, TimestampFirst);
  Graph.addState(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, TimestampFirst);
  Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, TimestampFirst);
  Graph.addState(CLOCK_DRIFT_STATE, libRSF::DataType::ClockDrift, TimestampFirst);

  /** copy values to real graph */
  Graph.getStateData().getElement(POSITION_STATE, TimestampFirst).setMean(SimpleGraph.getStateData().getElement(POSITION_STATE, TimestampFirst).getMean());
  Graph.getStateData().getElement(CLOCK_ERROR_STATE, TimestampFirst).setMean(SimpleGraph.getStateData().getElement(CLOCK_ERROR_STATE, TimestampFirst).getMean());

  /** add first set of measurements */
  AddPseudorangeMeasurements(Graph, Measurements, Config, TimestampFirst);
}

/** @brief Adds a pseudorange measurement to the graph
 *
 * @param Graph reference to the factor graph object
 * @param Measurements reference to the dataset that contains the measurement
 * @param Config reference to the factor graph config object that specifies the motion model
 * @param Timestamp a double timestamp of the current position
 * @return nothing
 *
 */
void AddPseudorangeMeasurements(libRSF::FactorGraph &Graph,
                                libRSF::SensorDataSet &Measurements,
                                libRSF::FactorGraphConfig const &Config,
                                double Timestamp)
{
  libRSF::StateList ListPseudorange;
  libRSF::Data Pseudorange;
  libRSF::GaussianDiagonal<1> NoisePseudorange;

  ListPseudorange.add(POSITION_STATE, Timestamp);
  ListPseudorange.add(CLOCK_ERROR_STATE, Timestamp);

  int SatNumber = Measurements.countElement(libRSF::DataType::Pseudorange3, Timestamp);

  for(int SatCounter = 0; SatCounter < SatNumber; ++SatCounter)
  {
    /** get measurement */
    Pseudorange = Measurements.getElement(libRSF::DataType::Pseudorange3, Timestamp, SatCounter);

    /** add factor */
    switch(Config.GNSS.ErrorModel.Type)
    {
      case libRSF::ErrorModelType::Gaussian:
        NoisePseudorange.setStdDevDiagonal(Pseudorange.getStdDevDiagonal());
        Graph.addFactor<libRSF::FactorType::Pseudorange3_ECEF>(ListPseudorange, Pseudorange, NoisePseudorange);
        break;

      case libRSF::ErrorModelType::DCS:
        NoisePseudorange.setStdDevDiagonal(Pseudorange.getStdDevDiagonal());
        Graph.addFactor<libRSF::FactorType::Pseudorange3_ECEF>(ListPseudorange, Pseudorange, NoisePseudorange, new libRSF::DCSLoss(1.0));
        break;

      case libRSF::ErrorModelType::cDCE:
        NoisePseudorange.setStdDevDiagonal(libRSF::Matrix11::Ones());
        Graph.addFactor<libRSF::FactorType::Pseudorange3_ECEF>(ListPseudorange, Pseudorange, NoisePseudorange, new libRSF::cDCELoss(Pseudorange.getStdDevDiagonal()[0]));
        break;

      case libRSF::ErrorModelType::GMM:
        static libRSF::GaussianMixture<1> GMM;
        if (GMM.getNumberOfComponents() == 0)
        {
          if (Config.GNSS.ErrorModel.GMM.TuningType == libRSF::ErrorModelTuningType::VBI)
          {
            GMM.initSpread(2, 10);/**< number of components is unknown, so choose 2 */
          }
          else
          {
            GMM.initSpread(GMM_N, 10);/**< number of components is known */
          }
        }

        if (Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
        {
          static libRSF::MaxMix1 NoisePseudorangeMaxMix(GMM);
          Graph.addFactor<libRSF::FactorType::Pseudorange3_ECEF>(ListPseudorange, Pseudorange, NoisePseudorangeMaxMix);
        }
        else if (Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
        {
          static libRSF::SumMix1 NoisePseudorangeSumMix(GMM);
          Graph.addFactor<libRSF::FactorType::Pseudorange3_ECEF>(ListPseudorange, Pseudorange, NoisePseudorangeSumMix);
        }
        else
        {
          PRINT_ERROR("Wrong error model mixture type!");
        }

        break;

      default:
        PRINT_ERROR("Wrong error model type: ", Config.GNSS.ErrorModel.Type);
        break;
    }
  }
}

/** @brief Use a GMM to estimate the error distribution of a factor
*
* @param Graph reference to the factor graph object
* @param Config reference to the factor graph config object that specifies the motion model
* @return nothing
*
 */
void TuneErrorModel(libRSF::FactorGraph &Graph,
                    libRSF::FactorGraphConfig &Config)
{
  if(Config.GNSS.ErrorModel.GMM.TuningType != libRSF::ErrorModelTuningType::None)
  {
    /** compute residuals of the factor graph */
    libRSF::Matrix ErrorData;
    Graph.computeUnweightedErrorMatrix(libRSF::FactorType::Pseudorange3_ECEF, ErrorData);

    if(Config.GNSS.ErrorModel.GMM.TuningType == libRSF::ErrorModelTuningType::EM)
    {
      /** fill empty GMM */
      libRSF::GaussianMixture<1> GMM;

      if(GMM.getNumberOfComponents() == 0)
      {
        GMM.initSpread(GMM_N, 10);
      }

      /** call the EM algorithm */
      libRSF::GaussianMixture<1>::EstimationConfig GMMConfig;
      GMMConfig.EstimationAlgorithm = libRSF::ErrorModelTuningType::EM;
      GMMConfig.RemoveSmallComponents = false;
      GMMConfig.MergeSimilarComponents = false;
      GMM.estimate(ErrorData, GMMConfig);

      /** remove offset of the first "LOS" component */
      GMM.removeOffsetLegacy();

      /** apply error model */
      if(Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
      {
        libRSF::SumMix1 NewSMModel(GMM);
        Graph.setNewErrorModel(libRSF::FactorType::Pseudorange3_ECEF, NewSMModel);
      }
      else if(Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
      {
        libRSF::MaxMix1 NewMMModel(GMM);
        Graph.setNewErrorModel(libRSF::FactorType::Pseudorange3_ECEF, NewMMModel);
      }

    }
    else if(Config.GNSS.ErrorModel.GMM.TuningType == libRSF::ErrorModelTuningType::VBI)
    {
      /** initialize GMM */
      static libRSF::GaussianMixture<1> GMMAdaptive;
      libRSF::GaussianComponent<1> Component;

      if(GMMAdaptive.getNumberOfComponents() == 0)
      {
        GMMAdaptive.initSpread(1, 10);
      }

      /** calculate statistics for GMM initialization*/
      const libRSF::Matrix11 Cov = libRSF::EstimateSampleCovariance<1>(ErrorData);

      /** add just one component per timestamp */
      if(GMMAdaptive.getNumberOfComponents() >= VBI_N_MAX)
      {
        GMMAdaptive.sortComponentsByWeight();
        GMMAdaptive.removeLastComponent();
      }

      Component.setParamsCovariance(Cov,
                                    libRSF::Vector1::Zero(),
                                    libRSF::Vector1::Ones()/GMMAdaptive.getNumberOfComponents());

      GMMAdaptive.addComponent(Component);

      /** estimate GMM with Variational Inference */
      libRSF::GaussianMixture<1>::EstimationConfig GMMConfig;
      GMMConfig.EstimationAlgorithm = libRSF::ErrorModelTuningType::VBI;
      GMMConfig.RemoveSmallComponents = true;
      GMMConfig.MergeSimilarComponents = false;
      GMMConfig.PriorWishartDOF = VBI_NU;
      GMMAdaptive.estimate(ErrorData, GMMConfig);

      /** remove offset*/
      GMMAdaptive.removeOffsetLegacy();

      /** apply error model */
      if(Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
      {
        libRSF::SumMix1 NewSMModel(GMMAdaptive);
        Graph.setNewErrorModel(libRSF::FactorType::Pseudorange3_ECEF, NewSMModel);
      }
      else if(Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
      {
        libRSF::MaxMix1 NewMMModel(GMMAdaptive);
        Graph.setNewErrorModel(libRSF::FactorType::Pseudorange3_ECEF, NewMMModel);
      }
    }
  }
}

bool ParseErrorModel(const std::string &ErrorModel, libRSF::FactorGraphConfig &Config)
{
  if(ErrorModel == "gauss")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::Gaussian;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "dcs")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::DCS;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "cdce")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::cDCE;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "sm")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::SumMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "mm")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::MaxMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "stsm")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::SumMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::EM;
  }
  else if(ErrorModel == "stmm")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::MaxMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::EM;
  }
  else if(ErrorModel == "stsm_vbi")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::SumMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::VBI;
  }
  else if(ErrorModel == "stmm_vbi")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::MaxMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::VBI;
  }
  else
  {
    PRINT_ERROR("Wrong Error Model: ", ErrorModel);
    return false;
  }

  return true;
}

int CreateGraphAndSolve(std::vector<std::string> &Arguments,
                        libRSF::StateDataSet &Result,
                        std::string &OutputFile)
{
  libRSF::FactorGraphConfig Config;

  /** read filenames */
  Config.InputFile = Arguments.at(0);
  Config.OutputFile = Arguments.at(1);
  OutputFile = Config.OutputFile; // for call by reference "return-value"

  /** parse the error model string */
  if (!ParseErrorModel(Arguments.at(3), Config))
  {
    return 1;
  }

  /** configure the solver */
  ceres::Solver::Options SolverOptions;
  SolverOptions.minimizer_progress_to_stdout = false;
  SolverOptions.use_nonmonotonic_steps = true;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  SolverOptions.max_num_iterations = 100;

  /** read input data */
  libRSF::SensorDataSet InputData;
  libRSF::ReadDataFromFile(Config.InputFile, InputData);

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;

  double Timestamp, TimestampFirst = 0.0, TimestampOld, TimestampLast;
  InputData.getTimeFirst(libRSF::DataType::Pseudorange3, TimestampFirst);
  InputData.getTimeLast(libRSF::DataType::Pseudorange3, TimestampLast);
  Timestamp = TimestampFirst;
  TimestampOld = TimestampFirst;
  int nTimestamp = 0;

  /** add fist variables and factors */
  InitGraph(Graph, InputData, Config, SolverOptions, TimestampFirst);

  /** solve multiple times with refined model to achieve good initial convergence */
  Graph.solve(SolverOptions);
  TuneErrorModel(Graph, Config);
  Graph.solve(SolverOptions);

  /** safe result at first timestamp */
  Result.addElement(POSITION_STATE, Graph.getStateData().getElement(POSITION_STATE, Timestamp, 0));

  /** get odometry noise from first measurement */
  libRSF::Data Odom = InputData.getElement(libRSF::DataType::Odom3, Timestamp);
  libRSF::Vector4 StdOdom4DOF;
  StdOdom4DOF << Odom.getStdDevDiagonal().head(3), Odom.getStdDevDiagonal().tail(1);
  libRSF::GaussianDiagonal<4> NoiseOdom4DOF;
  NoiseOdom4DOF.setStdDevDiagonal(StdOdom4DOF);

  /** hard coded constant clock error drift (CCED) model noise properties */
  libRSF::Vector2 StdCCED;

  if(Config.InputFile == "Chemnitz_Input.txt")
  {
    StdCCED << 0.1, 0.009; /** set CCED standard deviation for Chemnitz City dataset */
  }
  else
  {
    StdCCED << 0.05, 0.01; /** set CCED standard deviation for smartLoc datasets */
  }
  libRSF::GaussianDiagonal<2> NoiseCCED;
  NoiseCCED.setStdDevDiagonal(StdCCED);

  /** iterate over timestamps */
  while(InputData.getTimeNext(libRSF::DataType::Pseudorange3, Timestamp, Timestamp))
  {
    /** add position, orientation and clock error */
    Graph.addState(POSITION_STATE, libRSF::DataType::Point3, Timestamp);
    Graph.addState(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, Timestamp);
    Graph.addState(ORIENTATION_STATE, libRSF::DataType::Angle, Timestamp);
    Graph.addState(CLOCK_DRIFT_STATE, libRSF::DataType::ClockDrift, Timestamp);

    /** add odometry */
    libRSF::StateList MotionList;
    MotionList.add(POSITION_STATE, TimestampOld);
    MotionList.add(ORIENTATION_STATE, TimestampOld);
    MotionList.add(POSITION_STATE, Timestamp);
    MotionList.add(ORIENTATION_STATE, Timestamp);
    Graph.addFactor<libRSF::FactorType::Odom4_ECEF>(MotionList, InputData.getElement(libRSF::DataType::Odom3, Timestamp), NoiseOdom4DOF);

    /** add clock drift model */
    libRSF::StateList ClockList;
    ClockList.add(CLOCK_ERROR_STATE, TimestampOld);
    ClockList.add(CLOCK_DRIFT_STATE, TimestampOld);
    ClockList.add(CLOCK_ERROR_STATE, Timestamp);
    ClockList.add(CLOCK_DRIFT_STATE, Timestamp);
    Graph.addFactor<libRSF::FactorType::ConstDrift1>(ClockList, NoiseCCED);

    /** add all pseudo range measurements of with current timestamp */
    AddPseudorangeMeasurements(Graph, InputData, Config, Timestamp);

    /** tune self-tuning error model */
    TuneErrorModel(Graph, Config);

    /** solve the estimation problem */
    Graph.solve(SolverOptions);

    /** save data after optimization */
    Result.addElement(POSITION_STATE, Graph.getStateData().getElement(POSITION_STATE, Timestamp, 0));

    /** apply sliding window */
    Graph.removeAllStatesOutsideWindow(60, Timestamp);

    /** save time stamp */
    TimestampOld = Timestamp;

    nTimestamp++;
  }

  /** print last report */
  Graph.printReport();

  return 0;
}

double RMSE(const libRSF::Vector &V)
{
  return sqrt(V.squaredNorm() / static_cast<double>(V.size()));
}

double MyATE(const libRSF::DataType TypeGT,
           const libRSF::SensorDataSet &GT,
           const std::string &TypeEstimate,
           const libRSF::StateDataSet &Estimate)
{
  /** check length */
  const int LengthGT = GT.countElements(TypeGT);
  const int LengthEstimate = Estimate.countElements(TypeEstimate);

  /** return error if not equal */
  if (LengthGT != LengthEstimate)
  {
    PRINT_ERROR("Length of GT and estimate is not identical!");
    return std::numeric_limits<double>::quiet_NaN();
  }

  /** vector to store errors */
  libRSF::Vector Error(LengthEstimate);

  /** get first timestamp */
  double Time = 0.0;
  GT.getTimeFirst(TypeGT, Time);

  /** fill error vector */
  int n = 0;
  do
  {
    /** get data at this timestamp */
    libRSF::Data DataGT, DataEstimate;
    Estimate.getElement(TypeEstimate, Time, 0, DataEstimate);
    GT.getElement(TypeGT, Time, 0, DataGT);

    /** euclidean distance*/
    Error(n) = (DataEstimate.getMean() - DataGT.getMean()).norm();
    n++;
  }
  while(GT.getTimeNext(TypeGT, Time, Time));

  /** apply RMSE */
  return libRSF::RMSE(Error);
}

bool RunGNSS(const std::string& DataSet,
             const std::string& ErrorModel,
             libRSF::StateDataSet &Result,
             libRSF::SensorDataSet &GT)
{
  /** load ground truth */
  libRSF::ReadDataFromFile(std::string(DataSet) + std::string("_GT.txt"), GT);

  /** assign all arguments to string vector*/
  std::vector<std::string> Arguments;
  Arguments.push_back(std::string(DataSet) + std::string("_Input.txt"));
  Arguments.emplace_back("Result.txt"); // shouldn't be necessary, not writing
  Arguments.emplace_back("error:");
  Arguments.push_back(ErrorModel);

  /** dummy argument */
  std::string OutputFile;

  /** run optimization */
  return CreateGraphAndSolve(Arguments, Result, OutputFile) > 0;
}

int main() {
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  RunGNSS(DATA_BPB, ERROR_VBISM, Result, GT);

  /** calculate RMSE */
  const double ATE = MyATE(libRSF::DataType::Point3, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
  return 0;
}
