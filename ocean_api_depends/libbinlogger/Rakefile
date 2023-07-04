
task :default => "hunter:test"

namespace :hunter do
  task :build do
    sh "cmake -H. -B_builds -DHUNTER_STATUS_DEBUG=ON -DCMAKE_BUILD_TYPE=Release"
    sh "cmake --build _builds --config Release"
  end

  task :test do
    sh "cmake -H. -B_builds -DHUNTER_STATUS_DEBUG=ON -DBUILD_UNIT_TESTS=ON -DCMAKE_BUILD_TYPE=Release"
    sh "cmake --build _builds --config Release"

    sh "cd _builds && make unit_test"
  end
end



namespace :dependencies do

  task :trusty do
    sh "sudo apt-get install -y cmake libopencv-dev libtclap-dev libboost-all-dev zlib1g-dev"
    sh "pip install conan"
  end

  task :osx do
    sh "brew update"
    sh "brew tap homebrew/science"
    sh "brew install homebrew/science/opencv tclap libtool autoconf"
    sh "pip install conan"
  end

  namespace :travis do

    task :linux => "dependencies:trusty"

    task :osx => [:pip_uninstall_numpy, "dependencies:osx"] do
      sh "sudo ln -s /usr/local/bin/glibtoolize /usr/local/bin/libtoolize"
    end

    task :pip_uninstall_numpy do
      sh "pip uninstall -y numpy"
    end

  end
end
