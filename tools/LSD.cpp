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
#include <util/validators.hpp>

#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "util/Configuration.h"

#include "App/InputThread.h"
#include <boost/program_options.hpp>
#include <libvideoio/Undistorter.h>
#include <filesystem>
using namespace lsd_slam;
namespace fs=std::filesystem;
namespace bpo = boost::program_options;

int main( int argc, char** argv )
{
  libg3logger::G3Logger logWorker( argv[0] );
  logWorker.logBanner();

  bpo::options_description opt_desc{"Options"};

  // Add new options/flags here
  std::string calibFile;
  bool verbose;
  std::vector<std::string> inFiles;

  opt_desc.add_options()
    ("help,h", "Help")
    ("calib,c", bpo::value<decltype(calibFile)>(&calibFile)->required()->notifier(ExistingFile), "Calibration file")
    ("verbose,v", bpo::value<decltype(verbose)>(&verbose)->implicit_value(true)->default_value(false), "Print DEBUG output to console")
    ("input,i", bpo::value<decltype(inFiles)>(&inFiles)->multitoken()->composing(), "Input files or directories");

  bpo::variables_map vm;
  bpo::store(bpo::parse_command_line(argc, argv, opt_desc), vm);
  bpo::notify(vm);
  
  
  auto pUndisorter = libvideoio::UndistorterFactory::getUndistorterFromFile(calibFile);
  std::shared_ptr<libvideoio::Undistorter> undistorter(pUndisorter);
  // Load the configuration object
  Conf().setSlamImageSize( undistorter->outputImageSize() );
  // Conf().camera     = args.undistorter->getCamera();

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
    for(const auto& p : files)
    {
        vec_of_files.emplace_back(p);
    }
  LOG(INFO) << "Starting input thread.";
  std::shared_ptr<libvideoio::ImageSource> dataSource(new libvideoio::ImageFilesSource(vec_of_files));
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
