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

#ifndef Xyce_N_DEV_PythonInterface_h
#define Xyce_N_DEV_PythonInterface_h

#include <string>
#include <memory>
#include <pybind11/embed.h>

namespace Xyce {
namespace Device {

/**
 * @brief Singleton class to manage the embedded Python interpreter.
 */
class PythonInterface {
public:
    static PythonInterface& getInstance();
    
    // Non-copyable
    PythonInterface(const PythonInterface&) = delete;
    PythonInterface& operator=(const PythonInterface&) = delete;

    /**
     * @brief Creates a Python object of a specified class from a module.
     * @param moduleName Name of the Python module.
     * @param className Name of the class in the module.
     * @return pybind11::object The instantiated Python object.
     */
    pybind11::object createDevice(const std::string& moduleName, const std::string& className);
    void addSearchPath(const std::string& path);

    /**
     * @brief Check if initialized.
     */
    bool isInitialized() const { return guard_ != nullptr; }

private:
    PythonInterface();
    ~PythonInterface();
    
    std::unique_ptr<pybind11::scoped_interpreter> guard_;
};

} // namespace Device
} // namespace Xyce

#endif // Xyce_N_DEV_PythonInterface_h
