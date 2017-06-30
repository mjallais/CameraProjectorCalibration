##############################################################################
#
# Library:   CalibrationCameraProjector
#
# Author: Maeliss Jallais
#
# Copyright 2010 Kitware Inc. 28 Corporate Drive,
# Clifton Park, NY, 12065, USA.
#
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
##############################################################################

cmake_minimum_required( VERSION 3.5 )

project( CalibrationCameraProjector CXX )

list(INSERT CMAKE_MODULE_PATH 0 ${CalibrationCameraProjector_SOURCE_DIR}/cmake)

find_package(Qt5 COMPONENTS Core Widgets Gui OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(FlyCapture REQUIRED)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

set(CMAKE_AUTORCC ON)

set( source_files
  src/AboutDialog.cpp
  src/Application.cpp
  src/CalibrationData.cpp
  src/CalibrationDialog.cpp
  src/CaptureDialog.cpp
  src/GLWidget.cpp
  src/im_util.cpp
  src/ImageLabel.cpp
  src/io_util.cpp
  src/main.cpp
  src/MainWindow.cpp
  src/ProcessingDialog.cpp
  src/ProjectorWidget.cpp
  src/scan3d.cpp
  src/structured_light.cpp
  src/TreeModel.cpp
  src/VideoInput.cpp
  )

set( include_files
  include/AboutDialog.hpp
  include/Application.hpp
  include/CalibrationData.hpp
  include/CalibrationDialog.hpp
  include/CaptureDialog.hpp
  include/cognex_util.hpp
  include/GLWidget.hpp
  include/im_util.hpp
  include/ImageLabel.hpp
  include/io_util.hpp
  include/MainWindow.hpp
  include/ProcessingDialog.hpp
  include/ProjectorWidget.hpp
  include/scan3d.hpp
  include/structured_light.hpp
  include/TreeModel.hpp
  include/VideoInput.hpp
  include/VideoInput_QTkit.hpp
  )

qt5_wrap_ui( ui_files 
  ${CalibrationCameraProjector_SOURCE_DIR}/forms/AboutDialog.ui
  ${CalibrationCameraProjector_SOURCE_DIR}/forms/CalibrationDialog.ui
  ${CalibrationCameraProjector_SOURCE_DIR}/forms/CaptureDialog.ui
  ${CalibrationCameraProjector_SOURCE_DIR}/forms/MainWindow.ui
  ${CalibrationCameraProjector_SOURCE_DIR}/forms/ProcessingDialog.ui
   )
qt5_wrap_cpp( moc_files
  ${CalibrationCameraProjector_SOURCE_DIR}/include/AboutDialog.hpp
  ${CalibrationCameraProjector_SOURCE_DIR}/include/Application.hpp   
  ${CalibrationCameraProjector_SOURCE_DIR}/include/CalibrationDialog.hpp
  ${CalibrationCameraProjector_SOURCE_DIR}/include/CaptureDialog.hpp  
  ${CalibrationCameraProjector_SOURCE_DIR}/include/GLWidget.hpp
  ${CalibrationCameraProjector_SOURCE_DIR}/include/MainWindow.hpp
  ${CalibrationCameraProjector_SOURCE_DIR}/include/ProcessingDialog.hpp
  ${CalibrationCameraProjector_SOURCE_DIR}/include/ProjectorWidget.hpp
  ${CalibrationCameraProjector_SOURCE_DIR}/include/VideoInput.hpp
  )
  
set( qrc_files
  ${CalibrationCameraProjector_SOURCE_DIR}/assets/assets.qrc
  )

include_directories( ${CMAKE_CURRENT_BINARY_DIR} ${FLYCAPTURE_INCLUDE_DIR} include)

add_executable( CalibrationCameraProjector
  ${source_files}
  ${include_files}
  ${ui_files}
  ${moc_files}
  ${qrc_files}
  )

target_link_libraries( CalibrationCameraProjector
  Qt5::Widgets Qt5::Gui Qt5::OpenGL 
  ${OpenCV_LIBS} 
  ${FLYCAPTURE2_LIB}
  ${ITK_LIBRARIES}
  )
