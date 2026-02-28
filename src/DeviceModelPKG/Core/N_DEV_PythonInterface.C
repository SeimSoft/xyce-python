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

#include <N_DEV_PythonInterface.h>
#include <iostream>

namespace Xyce {
namespace Device {

PythonInterface& PythonInterface::getInstance() {
    static PythonInterface instance;
    return instance;
}

PythonInterface::PythonInterface() {
    try {
        if (!Py_IsInitialized()) {
            guard_.reset(new pybind11::scoped_interpreter());
            
            // Add current directory to path
            pybind11::module_ sys = pybind11::module_::import("sys");
            sys.attr("path").attr("append")(".");
            // Also add a common search path if needed, e.g. /Users/rennekef/Documents/XYCE-Python/python
            sys.attr("path").attr("append")("/Users/rennekef/Documents/XYCE-Python");
        }
    } catch (const pybind11::error_already_set& e) {
        std::cerr << "Python initialization failed: " << e.what() << std::endl;
    }
}

PythonInterface::~PythonInterface() {
    // scoped_interpreter will naturally shut down Python when destroyed
}

pybind11::object PythonInterface::createDevice(const std::string& moduleName, const std::string& className) {
    try {
        pybind11::module_ mod = pybind11::module_::import(moduleName.c_str());
        pybind11::object cls = mod.attr(className.c_str());
        return cls();
    } catch (const pybind11::error_already_set& e) {
        std::cerr << "Failed to create Python device " << moduleName << "." << className << ": " << e.what() << std::endl;
        return pybind11::none();
    }
}

} // namespace Device
} // namespace Xyce
