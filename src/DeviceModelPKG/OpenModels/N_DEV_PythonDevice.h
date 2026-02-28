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

class Model;
class Instance;

struct Traits : public DeviceTraits<Model, Instance>
{
    static const char *name() {return "Python Device";}
    static const char *deviceTypeName() {return "NPY level 1 (Python Device)";};
    static int numNodes() {return 2;}
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
  virtual bool getBreakPoints(std::vector<Util::BreakPoint> & breakPointTimes,
                             std::vector<Util::BreakPoint> & pauseBreakPointTimes);

  virtual void loadNodeSymbols(Util::SymbolTable &symbol_table) const override;

private:
  std::string moduleName_;
  std::string className_;
  pybind11::object pyDevice_;
  
  Model & model_;
  
  std::vector<int> extLIDs_;
  std::vector<std::vector<int>> jacLIDs_;
  std::vector<std::vector<int>> jacStamp_;
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
  std::vector<Instance*> instanceContainer;
};

void registerDevice(const DeviceCountMap& deviceMap, const std::set<int>& levelSet);

} // namespace PythonDevice
} // namespace Device
} // namespace Xyce

#endif // Xyce_N_DEV_PythonDevice_h
