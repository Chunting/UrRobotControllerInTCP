#include "ForceLogicIO.h"
ForceLogicIO::ForceLogicIO()
{
    
}

bool ForceLogicIO::setup(const QString& configFilePath) {
    //COBOT_LOG.message() << "Use default config.";
    return true;
}

bool ForceLogicIO::waitforLogicIOEnd(void)
{
    return false;
}

bool ForceLogicIO::start()
{
    return waitforLogicIOEnd();
    //return false;
}

void ForceLogicIO::stop()
{
    //waitforLogicIOEnd();
    //return false;
}

void ForceLogicIO::onCallbackPosition(void)
{
    
}

void ForceLogicIO::unpack(void)
{
    
}

void ForceLogicIO::pause()
{
    
}
