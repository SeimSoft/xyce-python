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
  addPar("MODULE", std::string(""), &PythonDevice::Model::moduleName_);
  addPar("CLASS", std::string(""), &PythonDevice::Model::className_);
  addPar("PYTHONPATH", std::string(""), &PythonDevice::Model::pythonPath_);
}

template<>
ParametricData<PythonDevice::Instance>::ParametricData()
{
}

namespace PythonDevice {

namespace {
Instance* g_activeInstance = nullptr;
}

Input::Input(int index) : index_(index), voltage_(0.0) {
    if (g_activeInstance) g_activeInstance->registerInput(this);
}

ResistorOutput::ResistorOutput(int index, double r, Input* vhigh, Input* vlow)
    : index_(index), r_(r), vhigh_(vhigh), vlow_(vlow), state_(0), current_(0.0) {
    if (g_activeInstance) g_activeInstance->registerResistorOutput(this);
}

VoltageOutput::VoltageOutput(int index)
    : index_(index), startValue_(0.0), targetValue_(0.0), startTime_(0.0), dt_(0.0), inTransition_(false) {
    if (g_activeInstance) g_activeInstance->registerVoltageOutput(this);
}

void VoltageOutput::transition_to(double v, double dt, double currentTime) {
    if (dt <= 0) {
        set_value(v);
        return;
    }
    startValue_ = get_current_value(currentTime);
    targetValue_ = v;
    startTime_ = currentTime;
    dt_ = dt;
    inTransition_ = true;
}

double VoltageOutput::get_current_value(double t) const {
    if (!inTransition_ || t <= startTime_) return startValue_;
    if (t >= startTime_ + dt_) return targetValue_;
    return startValue_ + (targetValue_ - startValue_) * ((t - startTime_) / dt_);
}

CurrentOutput::CurrentOutput(int index)
    : index_(index), startValue_(0.0), targetValue_(0.0), startTime_(0.0), dt_(0.0), inTransition_(false) {
    if (g_activeInstance) g_activeInstance->registerCurrentOutput(this);
}

void CurrentOutput::transition_to(double i, double dt, double currentTime) {
    if (dt <= 0) {
        set_value(i);
        return;
    }
    startValue_ = get_current_value(currentTime);
    targetValue_ = i;
    startTime_ = currentTime;
    dt_ = dt;
    inTransition_ = true;
}

double CurrentOutput::get_current_value(double t) const {
    if (!inTransition_ || t <= startTime_) return startValue_;
    if (t >= startTime_ + dt_) return targetValue_;
    return startValue_ + (targetValue_ - startValue_) * ((t - startTime_) / dt_);
}


int ResistorOutput::get_vhigh_node() const {
    return vhigh_ ? vhigh_->get_index() : -1;
}

int ResistorOutput::get_vlow_node() const {
    return vlow_ ? vlow_->get_index() : -1;
}

PYBIND11_EMBEDDED_MODULE(xyce_device, m) {
    pybind11::class_<Input>(m, "Input")
        .def(pybind11::init<int>())
        .def("get_v", &Input::get_v);
    pybind11::class_<ResistorOutput>(m, "ResistorOutput")
        .def(pybind11::init<int, double, Input*, Input*>())
        .def("set_state", &ResistorOutput::set_state)
        .def("get_i", &ResistorOutput::get_i);
    pybind11::class_<VoltageOutput>(m, "VoltageOutput")
        .def(pybind11::init<int>())
        .def("set_value", &VoltageOutput::set_value)
        .def("transition_to", [](VoltageOutput& vo, double v, double dt){
            if (g_activeInstance) {
                vo.transition_to(v, dt, g_activeInstance->getSolverState().currTime_);
                // Auto-register breakpoint for end of transition
                g_activeInstance->addNextBreakpoint(g_activeInstance->getSolverState().currTime_ + dt);
            }
        }, pybind11::arg("v"), pybind11::arg("dt"));
    pybind11::class_<CurrentOutput>(m, "CurrentOutput")
        .def(pybind11::init<int>())
        .def("set_value", &CurrentOutput::set_value)
        .def("transition_to", [](CurrentOutput& co, double i, double dt){
            if (g_activeInstance) {
                co.transition_to(i, dt, g_activeInstance->getSolverState().currTime_);
                g_activeInstance->addNextBreakpoint(g_activeInstance->getSolverState().currTime_ + dt);
            }
        }, pybind11::arg("i"), pybind11::arg("dt"));
}




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
{
    setDefaultParams();
    setParams(model_block.params);
    updateDependentParameters();
}

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
    model_(model),
    lastUpdateTime_(-1.0)
{
    numExtVars = instance_block.numExtVars;
    numIntVars = 0;
    numStateVars = 0;

    setDefaultParams();
    setParams(instance_block.params);
    updateDependentParameters();

    // Initialize Python device
    g_activeInstance = this;
    if (!model.pythonPath_.empty()) {
        PythonInterface::getInstance().addSearchPath(model.pythonPath_);
    }
    if (!model.moduleName_.empty() && !model.className_.empty()) {
        pyDevice_ = PythonInterface::getInstance().createDevice(model.moduleName_, model.className_);
        if (pyDevice_ && !pyDevice_.is_none()) {
            pyDevice_.attr("set_next_breakpoint") = pybind11::cpp_function([this](double t){
                this->addNextBreakpoint(t);
            });
        }
    }
    g_activeInstance = nullptr;

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
        
        // Call update if time has changed (start of new step or DC)
        if (time != lastUpdateTime_) {
            // Update input voltages for Python to read
            for (auto& in : inputs_) {
                int lid = extLIDs_[in->get_index()];
                if (lid >= 0) in->set_voltage((*extData.nextSolVectorPtr)[lid]);
                else in->set_voltage(0.0);
            }
            
            g_activeInstance = this;
            if (pybind11::hasattr(pyDevice_, "update")) {
                pyDevice_.attr("update")(time);
            }
            g_activeInstance = nullptr;
            lastUpdateTime_ = time;
        }


        // Load residuals from resistor outputs
        for (auto& ro : resistorOutputs_) {
            int outIdx = ro->get_index();
            int outLid = extLIDs_[outIdx];
            if (outLid < 0) continue;
            
            int state = ro->get_state();
            int srcIdx = (state == 1) ? ro->get_vhigh_node() : ro->get_vlow_node();
            int srcLid = (srcIdx >= 0) ? extLIDs_[srcIdx] : -1;
            
            double vOut = (*extData.nextSolVectorPtr)[outLid];
            double vSrc = (srcLid >= 0) ? (*extData.nextSolVectorPtr)[srcLid] : 0.0;
            
            double i = (vOut - vSrc) / ro->get_r();
            ro->set_current(i);
            
            (*extData.daeFVectorPtr)[outLid] += i;
            if (srcLid >= 0) {
                (*extData.daeFVectorPtr)[srcLid] -= i;
            }
        }
        
        // Load voltage outputs (penalty method)
        for (auto& vo : voltageOutputs_) {
            int lid = extLIDs_[vo->get_index()];
            if (lid >= 0) {
                double v = (*extData.nextSolVectorPtr)[lid];
                double g = 1e6; // Conductance (stiff source)
                (*extData.daeFVectorPtr)[lid] += (v - vo->get_current_value(time)) * g;
            }
        }
        
        // Load current outputs
        for (auto& co : currentOutputs_) {
            int lid = extLIDs_[co->get_index()];
            if (lid >= 0) {
                (*extData.daeFVectorPtr)[lid] += co->get_current_value(time);
            }
        }

    } catch (const pybind11::error_already_set& e) {
        std::cerr << "Python update failed: " << e.what() << std::endl;
    }


    return true;
}

bool Instance::loadDAEdFdx()
{
    if (!pyDevice_ || pyDevice_.is_none()) return true;

    for (auto& ro : resistorOutputs_) {
        int outIdx = ro->get_index();
        int outLid = extLIDs_[outIdx];
        if (outLid < 0) continue;
        
        int state = ro->get_state();
        int srcIdx = (state == 1) ? ro->get_vhigh_node() : ro->get_vlow_node();
        int srcLid = (srcIdx >= 0) ? extLIDs_[srcIdx] : -1;
        
        double g = 1.0 / ro->get_r();
        
        // dIout/dVout
        (*extData.dFdxMatrixPtr)[outLid][jacLIDs_[outIdx][outIdx]] += g;
        
        if (srcLid >= 0) {
            // dIout/dVsrc
            (*extData.dFdxMatrixPtr)[outLid][jacLIDs_[outIdx][srcIdx]] -= g;
            // dIsrc/dVout
            (*extData.dFdxMatrixPtr)[srcLid][jacLIDs_[srcIdx][outIdx]] -= g;
            // dIsrc/dVsrc
            (*extData.dFdxMatrixPtr)[srcLid][jacLIDs_[srcIdx][srcIdx]] += g;
        }
    }
    
    // Voltage outputs Jacobian
    for (auto& vo : voltageOutputs_) {
        int idx = vo->get_index();
        int lid = extLIDs_[idx];
        if (lid >= 0) {
            double g = 1e6;
            (*extData.dFdxMatrixPtr)[lid][jacLIDs_[idx][idx]] += g;
        }
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
    for (double t : nextBreakpoints_) {
        breakPointTimes.push_back(Util::BreakPoint(t));
    }
    nextBreakpoints_.clear();
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

  //std::cout << "DEBUG PythonDevice::registerDevice: deviceMap size=" << deviceMap.size() << std::endl;
  //if (deviceMap.find("N") != deviceMap.end()) std::cout << "DEBUG PythonDevice::registerDevice: 'N' found in deviceMap" << std::endl;

  if (!initialized && (deviceMap.empty() || (deviceMap.find("N")!=deviceMap.end()))) 
  {
    initialized = true;
    //std::cout << "DEBUG PythonDevice::registerDevice: Registering 'n' and 'npy'" << std::endl;

    Config<Traits>::addConfiguration()
      .registerDevice("n", 1)
      .registerModelType("npy", 1);
  }
}

} // namespace PythonDevice
} // namespace Device
} // namespace Xyce
