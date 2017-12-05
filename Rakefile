
require 'pathname'

# Rake support files are stored in the .rake subdirectory
$:.unshift File.dirname(__FILE__) + "/.rake"
require 'docker'
require 'dependencies'
require 'benchmark'
require 'build'
require 'build_tasks'

##
@coverity_email = ENV['LSDSLAM_COVERITY_EMAIL']
@coverity_token = ENV['LSDSLAM_COVERITY_TOKEN']

@build_parallelism = nil

## Any of the configuration variables given above can be
## overridden in a config.rb file
load 'config.rb' if FileTest::exists? 'config.rb'

## Builds occur in directories "#{BUILD_ROOT}-#{build_type}"
## e.g. build-debug/, build-release/
build_root = ENV['BUILD_ROOT'] || "build"

cmake = CMake.new

newBuilds = [ Build.new( "Debug", cmake: cmake  ),
              Build.new( "Debug_NoGUI", gui: false, cmake: cmake ),
              Build.new( "Release", cmake: cmake  )
            ]
BuildTasks.new( newBuilds )

task :default => "debug:test"

DockerTasks.new( builds: %w( Release Debug Debug_GUI ) )
BenchmarkTasks.new( newBuilds )
