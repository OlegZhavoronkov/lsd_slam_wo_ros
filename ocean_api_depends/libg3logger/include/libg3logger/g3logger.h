#pragma once

#include <g3log/logworker.hpp>

#include "ColorStderrSink.h"

namespace libg3logger {

  struct G3Logger {

    G3Logger( const std::string &appName, const LEVELS defaultLevel = WARNING );
    ~G3Logger();

    void logBanner( void );

    void verbose( bool );
    void setLevel( const LEVELS level );

    std::unique_ptr<g3::LogWorker> worker;
    std::unique_ptr<g3::SinkHandle<ColorStderrSink>> stderrHandle;
  };


}
