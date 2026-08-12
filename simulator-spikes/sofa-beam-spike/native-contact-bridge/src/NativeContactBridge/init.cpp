#include <NativeContactBridge/init.h>
#include <NativeContactBridge/NativeContactBridge.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>

namespace nativecontactbridge
{
extern "C"
{
NATIVE_CONTACT_BRIDGE_API void initExternalModule();
NATIVE_CONTACT_BRIDGE_API const char* getModuleName();
NATIVE_CONTACT_BRIDGE_API const char* getModuleVersion();
NATIVE_CONTACT_BRIDGE_API const char* getModuleLicense();
NATIVE_CONTACT_BRIDGE_API const char* getModuleDescription();
NATIVE_CONTACT_BRIDGE_API void registerObjects(sofa::core::ObjectFactory* factory);
}

void initExternalModule() { init(); }
const char* getModuleName() { return "NativeContactBridge"; }
const char* getModuleVersion() { return "0.1.0"; }
const char* getModuleLicense() { return "LGPL"; }
const char* getModuleDescription() { return "Minimal bridge exposing SOFA native narrow-phase contact outputs as Data."; }
void registerObjects(sofa::core::ObjectFactory* factory) { registerNativeContactBridge(factory); }

void init()
{
    static bool first = true;
    if (first)
    {
        sofa::helper::system::PluginManager::getInstance().registerPlugin("NativeContactBridge");
        first = false;
    }
}
}
