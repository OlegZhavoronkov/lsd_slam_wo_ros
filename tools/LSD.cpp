/**
*  This version "LSD" of the main binary has no GUI dependencies.
*  Pangolin, etc are not required to build this version.
*  (of course, it's not that exciting to watch!)
*
*  See LSD_GUI which is functionally identical but does have the
*  Pangolin-based GUI
*
* Based on original LSD-SLAM code from:
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <thread>

#include "libg3logger/g3logger.h"

#include "SlamSystem.h"

#include "util/settings.h"

#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "util/Configuration.h"
#include <CLI/Error.hpp>
#include "CLI/App.hpp"

#include "App/InputThread.h"
#include "CLI/CLI.hpp"
#include <libvideoio/Undistorter.h>
#include <filesystem>
using namespace lsd_slam;
namespace fs=std::filesystem;

int main( int argc, char** argv )
{
  libg3logger::G3Logger logWorker( argv[0] );
  logWorker.logBanner();

  CLI::App app;

  // Add new options/flags here
  std::string calibFile;
  app.add_option("-c,--calib", calibFile, "Calibration file" )->required()->check(CLI::ExistingFile);

  bool verbose;
  app.add_flag("-v,--verbose", verbose, "Print DEBUG output to console");

  std::vector<std::string> inFiles;
  app.add_option("--input,input", inFiles, "Input files or directories");

  // Defines the configuration file;  see
  //    https://cliutils.gitlab.io/CLI11Tutorial/chapters/config.html
  app.set_config("--config");

  CLI11_PARSE(app, argc, argv);
  lsd_slam::Conf().debugDisplay=1;
  lsd_slam::Conf().runRealTime=false;
    auto pUndisorter=libvideoio::UndistorterFactory::getUndistorterFromFile(calibFile);
    std::shared_ptr<libvideoio::Undistorter> undistorter(pUndisorter);
  // Load the configuration object
  Conf().setSlamImageSize( undistorter->outputImageSize() );
  // Conf().camera     = args.undistorter->getCamera();
  lsd_slam::plotTrackingIterationInfo=true;
  lsd_slam::plotSim3TrackingIterationInfo=true;
  lsd_slam::plotStereoImages=true;
  lsd_slam::plotTracking=true;

  LOG(INFO) << "Slam image: " << Conf().slamImageSize.width << " x " << Conf().slamImageSize.height;

  CHECK( (undistorter->getCamera().fx) > 0 && (undistorter->getCamera().fy > 0) ) << "Camera focal length is zero";

	std::shared_ptr<SlamSystem> system( new SlamSystem() );
    std::string folder=inFiles[0];
    CHECK(fs::is_directory(folder));
    std::list<std::string> files;
    for(const auto&p : fs::directory_iterator(folder))
    {
        if(!fs::is_regular_file(p.path()))
        {
            continue;
        }
        files.push_back(p.path().string());
    }
    files.sort();
    std::vector<std::string > vec_of_files;
    vec_of_files.reserve(files.size());
    size_t startIdx=500;
    size_t curr_idx=0;
    for(const auto& p : files)
    {
        if(curr_idx++ > startIdx)
        {
            vec_of_files.emplace_back(p);
        }
        
    }
  LOG(INFO) << "Starting input thread.";
  std::shared_ptr<libvideoio::ImageSource> dataSource(new libvideoio::ImageFilesSource(vec_of_files));
  
  dataSource->setFPS(0.5);
  InputThread input( system, dataSource, undistorter );
  std::thread inputThread( std::ref(input) );
  input.inputReady.wait();

  // Wait for all threads to be ready.
  LOG(INFO) << "Starting all threads.";
  startAll.notify();

  // This is idle loop
  while( !input.inputDone.getValue() )  { sleep(1); }

  LOG(INFO) << "Finalizing system.";
  system->finalize();

  return 0;
}
