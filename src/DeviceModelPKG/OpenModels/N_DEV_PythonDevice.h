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

#ifndef Xyce_N_DEV_PythonDevice_h
#define Xyce_N_DEV_PythonDevice_h

#include <N_DEV_fwd.h>
#include <N_DEV_Configuration.h>
#include <N_DEV_DeviceInstance.h>
#include <N_DEV_DeviceModel.h>
#include <N_DEV_DeviceMaster.h>
#include <N_DEV_PythonInterface.h>

namespace Xyce {
namespace Device {
namespace PythonDevice {

class Input {
public:
    Input(int index);
    double get_v() const { return voltage_; }
    void set_voltage(double v) { voltage_ = v; }
    int get_index() const { return index_; }
private:
    int index_;
    double voltage_;
};

class ResistorOutput {
public:
    ResistorOutput(int index, double r, Input* vhigh, Input* vlow);
    void set_state(int state) { state_ = state; }
    int get_state() const { return state_; }
    int get_index() const { return index_; }
    double get_r() const { return r_; }
    int get_vhigh_node() const;
    int get_vlow_node() const;
    double get_i() const { return current_; }
    void set_current(double i) { current_ = i; }
private:
    int index_;
    double r_;
    Input* vhigh_;
    Input* vlow_;
    int state_;
    double current_;
};

class VoltageOutput {
public:
    VoltageOutput(int index);
    void set_value(double v) {
        targetValue_ = v;
        startValue_ = v;
        dt_ = 0;
        inTransition_ = false;
    }
    void transition_to(double v, double dt, double currentTime);
    double get_current_value(double t) const;
    int get_index() const { return index_; }
private:
    int index_;
    double startValue_;
    double targetValue_;
    double startTime_;
    double dt_;
    bool inTransition_;
};

class CurrentOutput {
public:
    CurrentOutput(int index);
    void set_value(double i) {
        targetValue_ = i;
        startValue_ = i;
        dt_ = 0;
        inTransition_ = false;
    }
    void transition_to(double i, double dt, double currentTime);
    double get_current_value(double t) const;
    int get_index() const { return index_; }
private:
    int index_;
    double startValue_;
    double targetValue_;
    double startTime_;
    double dt_;
    bool inTransition_;
};

class Model;
class Instance;


struct Traits : public DeviceTraits<Model, Instance>
{
    static const char *name() {return "Python Device";}
    static const char *deviceTypeName() {return "NPY level 1 (Python Device)";};
    static int numNodes() {return 8;}
    static int numOptionalNodes() {return 0;}
    static bool isLinearDevice() {return false;}

    static Device *factory(const Configuration &configuration, const FactoryBlock &factory_block);
    static void loadModelParameters(ParametricData<Model> &model_parameters);
    static void loadInstanceParameters(ParametricData<Instance> &instance_parameters);
};

class Instance : public DeviceInstance
{
  friend class ParametricData<Instance>;
  friend class Model;
  friend struct Traits;
  friend class DeviceMaster<Traits>;

public:
  Instance(
    const Configuration &       configuration,
    const InstanceBlock &       instance_block,
    Model &                     model,
    const FactoryBlock &        factory_block);

  virtual ~Instance();

  virtual void registerLIDs( const std::vector<int> & intLIDVecRef,
                             const std::vector<int> & extLIDVecRef );
  
  virtual void registerJacLIDs( const std::vector< std::vector<int> > & jacLIDVec );
  virtual const std::vector< std::vector<int> > & jacobianStamp() const { return jacStamp_; }

  virtual bool updateIntermediateVars() { return true; }
  virtual bool updatePrimaryState() { return true; }

  virtual bool loadDAEFVector();
  virtual bool loadDAEdFdx();
  
  virtual bool loadDAEQVector() { return true; }
  virtual bool loadDAEdQdx() { return true; }
  
  virtual void acceptStep();
  virtual bool getInstanceBreakPoints(std::vector<Util::BreakPoint> & breakPointTimes);

  virtual void loadNodeSymbols(Util::SymbolTable &symbol_table) const override;

  void registerInput(Input* input) { inputs_.push_back(input); }
  void registerResistorOutput(ResistorOutput* ro) { resistorOutputs_.push_back(ro); }
  void registerVoltageOutput(VoltageOutput* vo) { voltageOutputs_.push_back(vo); }
  void registerCurrentOutput(CurrentOutput* co) { currentOutputs_.push_back(co); }
  void add_breakpoint(double t) { nextBreakpoints_.insert(t); }

private:
  pybind11::object pyDevice_;
  
  Model & model_;
  
  std::vector<int> extLIDs_;
  std::vector<std::vector<int>> jacLIDs_;
  std::vector<std::vector<int>> jacStamp_;

  std::vector<Input*> inputs_;
  std::vector<ResistorOutput*> resistorOutputs_;
  std::vector<VoltageOutput*> voltageOutputs_;
  std::vector<CurrentOutput*> currentOutputs_;
  std::vector<std::unique_ptr<VoltageOutput>> defaultVoltageOutputs_;
  std::set<double> nextBreakpoints_;
  double lastUpdateTime_;
};

class Model : public DeviceModel
{
    friend class ParametricData<Model>;
    friend class Instance;
    friend struct Traits;
    friend class DeviceMaster<Traits>;

public:
  Model(
    const Configuration &       configuration,
    const ModelBlock &          model_block,
    const FactoryBlock &        factory_block);
  virtual ~Model();

  virtual void forEachInstance(DeviceInstanceOp &op) const;
  virtual std::ostream &printOutInstances(std::ostream &os) const;

  void addInstance(Instance *instance) 
  {
    instanceContainer.push_back(instance);
  }

  virtual bool processInstanceParams();
  virtual bool processParams() { return true; }

private:
  std::string moduleName_;
  std::string className_;
  std::string pythonPath_;

  std::vector<Instance*> instanceContainer;
};

void registerDevice(const DeviceCountMap& deviceMap, const std::set<int>& levelSet);

} // namespace PythonDevice
} // namespace Device
} // namespace Xyce

#endif // Xyce_N_DEV_PythonDevice_h
