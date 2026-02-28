//-------------------------------------------------------------------------
//   Copyright 2002-2025 National Technology & Engineering Solutions of
//   Sandia, LLC (NTESS).  Under the terms of Contract DE-NA0003525 with
//   NTESS, the U.S. Government retains certain rights in this software.
//
//   This file is part of the Xyce(TM) Parallel Electrical Simulator.
//
//   Xyce(TM) is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   Xyce(TM) is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License
//   along with Xyce(TM).
//   If not, see <http://www.gnu.org/licenses/>.
//-------------------------------------------------------------------------

#include <N_DEV_PythonDevice.h>
#include <N_DEV_SolverState.h>
#include <N_DEV_Message.h>
#include <N_LAS_Vector.h>
#include <N_LAS_Matrix.h>
#include <N_UTL_BreakPoint.h>
#include <N_DEV_DeviceMaster.h>
#include <N_DEV_ExternData.h>

namespace Xyce {
namespace Device {

template<>
ParametricData<PythonDevice::Model>::ParametricData()
{
}

template<>
ParametricData<PythonDevice::Instance>::ParametricData()
{
  addPar("MODULE", std::string(""), &PythonDevice::Instance::moduleName_);
  addPar("CLASS", std::string(""), &PythonDevice::Instance::className_);
}

namespace PythonDevice {

Device *Traits::factory(const Configuration &configuration, const FactoryBlock &factory_block)
{
    return new DeviceMaster<Traits>(configuration, factory_block, factory_block.solverState_, factory_block.deviceOptions_);
}

void Traits::loadModelParameters(ParametricData<Model> &model_parameters)
{}

void Traits::loadInstanceParameters(ParametricData<Instance> &instance_parameters)
{}

Model::Model(const Configuration &configuration, const ModelBlock &model_block, const FactoryBlock &factory_block)
  : DeviceModel(model_block, configuration.getModelParameters(), factory_block)
{}

Model::~Model()
{}

void Model::forEachInstance(DeviceInstanceOp &op) const
{
  for (std::vector<Instance*>::const_iterator it = instanceContainer.begin(); it != instanceContainer.end(); ++it)
  {
    op(*it);
  }
}

std::ostream &Model::printOutInstances(std::ostream &os) const
{
  return os;
}

bool Model::processInstanceParams()
{
  for (std::vector<Instance*>::const_iterator it = instanceContainer.begin(); it != instanceContainer.end(); ++it)
  {
    (*it)->processParams();
  }
  return true;
}

Instance::Instance(const Configuration &configuration, const InstanceBlock &instance_block, Model &model, const FactoryBlock &factory_block)
  : DeviceInstance(instance_block, configuration.getInstanceParameters(), factory_block),
    model_(model)
{
    numExtVars = instance_block.numExtVars;
    numIntVars = 0;
    numStateVars = 0;

    setDefaultParams();
    setParams(instance_block.params);
    updateDependentParameters();

    // Initialize Python device
    if (!moduleName_.empty() && !className_.empty()) {
        pyDevice_ = PythonInterface::getInstance().createDevice(moduleName_, className_);
    }

    // Setup Jacobian stamp (all-to-all for python device)
    jacStamp_.resize(numExtVars);
    for (int i=0; i<numExtVars; ++i) {
        jacStamp_[i].resize(numExtVars);
        for (int j=0; j<numExtVars; ++j) {
            jacStamp_[i][j] = j;
        }
    }
    
    model.addInstance(this);
}

Instance::~Instance()
{
}

void Instance::registerLIDs(const std::vector<int> &intLIDVecRef, const std::vector<int> &extLIDVecRef)
{
    extLIDs_ = extLIDVecRef;
}

void Instance::registerJacLIDs(const std::vector<std::vector<int>> &jacLIDVec)
{
    jacLIDs_ = jacLIDVec;
}

bool Instance::loadDAEFVector()
{
    if (!pyDevice_ || pyDevice_.is_none()) return true;

    try {
        double time = getSolverState().currTime_;
        pybind11::list voltages;
        for (int i=0; i<numExtVars; ++i) {
            voltages.append((*extData.nextSolVectorPtr)[extLIDs_[i]]);
        }

        pybind11::list currents = pyDevice_.attr("evaluate_f")(time, voltages);
        
        for (int i=0; i<numExtVars; ++i) {
            (*extData.daeFVectorPtr)[extLIDs_[i]] -= currents[i].cast<double>();
        }
    } catch (const pybind11::error_already_set& e) {
        std::cerr << "Python evaluate_f failed: " << e.what() << std::endl;
    }

    return true;
}

bool Instance::loadDAEdFdx()
{
    if (!pyDevice_ || pyDevice_.is_none()) return true;

    try {
        double time = getSolverState().currTime_;
        pybind11::list voltages;
        for (int i=0; i<numExtVars; ++i) {
            voltages.append((*extData.nextSolVectorPtr)[extLIDs_[i]]);
        }

        pybind11::list jacobian = pyDevice_.attr("evaluate_jacobian")(time, voltages);
        
        for (int i=0; i<numExtVars; ++i) {
            pybind11::list row = jacobian[i].cast<pybind11::list>();
            for (int j=0; j<numExtVars; ++j) {
                int iRow = extLIDs_[i];
                int iCol = jacLIDs_[i][j];
                (*extData.dFdxMatrixPtr)[iRow][iCol] += row[j].cast<double>();
            }
        }
    } catch (const pybind11::error_already_set& e) {
        // Fallback or warning
    }

    return true;
}

void Instance::acceptStep()
{
    if (!pyDevice_ || pyDevice_.is_none()) return;

    try {
        double time = getSolverState().currTime_;
        pyDevice_.attr("accept_step")(time);
    } catch (const pybind11::error_already_set& e) {
        // Ignore or log
    }
}

bool Instance::getBreakPoints(std::vector<Util::BreakPoint> &breakPointTimes, std::vector<Util::BreakPoint> &pauseBreakPointTimes)
{
    if (!pyDevice_ || pyDevice_.is_none()) return true;

    try {
        pybind11::list pts = pyDevice_.attr("get_breakpoints")();
        for (auto item : pts) {
            breakPointTimes.push_back(Util::BreakPoint(item.cast<double>()));
        }
    } catch (const pybind11::error_already_set& e) {
        // Ignore
    }
    return true;
}

void Instance::loadNodeSymbols(Util::SymbolTable &symbol_table) const
{
  for (int i=0; i<numExtVars; ++i) {
      std::ostringstream oss;
      oss << "node" << i;
      addInternalNode(symbol_table, extLIDs_[i], getName(), oss.str());
  }
}

void registerDevice(const DeviceCountMap& deviceMap, const std::set<int>& levelSet) 
{
  static bool initialized = false;

  std::cout << "DEBUG PythonDevice::registerDevice: deviceMap size=" << deviceMap.size() << std::endl;
  if (deviceMap.find("N") != deviceMap.end()) std::cout << "DEBUG PythonDevice::registerDevice: 'N' found in deviceMap" << std::endl;

  if (!initialized && (deviceMap.empty() || (deviceMap.find("N")!=deviceMap.end()))) 
  {
    initialized = true;
    std::cout << "DEBUG PythonDevice::registerDevice: Registering 'n' and 'npy'" << std::endl;

    Config<Traits>::addConfiguration()
      .registerDevice("N", 101)
      .registerModelType("NPY", 1);
  }
}

} // namespace PythonDevice
} // namespace Device
} // namespace Xyce
