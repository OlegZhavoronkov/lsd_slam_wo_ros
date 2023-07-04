
#include <unistd.h>

#include <iostream>
#include <g3log/g3log.hpp>

#include "libg3logger/g3logger.h"


namespace libg3logger {

G3Logger::G3Logger( const std::string &appName, const LEVELS level )
  : worker( g3::LogWorker::createLogWorker() )
{
  auto handle = worker->addDefaultLogger(appName, ".");

  stderrHandle = worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
                                     &ColorStderrSink::ReceiveLogMessage);
  stderrHandle->call( &ColorStderrSink::setThreshold, level );

  g3::initializeLogging(worker.get());
  std::future<std::string> log_file_name = handle->call(&g3::FileSink::fileName);

  // This should be the only message written explicitly to std::cout
  // Everything else gets sent to the logger
  std::cout << "*\n*   Log file: [" << log_file_name.get() << "]\n*\n" << std::endl;

}


G3Logger::~G3Logger() {
  worker.release();

  g3::internal::shutDownLogging();
}

void G3Logger::verbose( bool verbose )
{
   stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );
}

void G3Logger::setLevel( const LEVELS level )
{
   stderrHandle->call( &ColorStderrSink::setThreshold, level );
}

void G3Logger::logBanner( void )
{

  LOG(INFO) << "Starting log.";

  #ifdef ENABLE_SSE
    LOG(INFO) << "With SSE optimizations.";
  #elif ENABLE_NEON
    LOG(INFO) << "With NEON optimizations.";
  #endif

}

}
