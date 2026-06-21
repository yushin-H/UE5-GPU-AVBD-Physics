#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"
#include "ShaderCore.h"

class FCustomPhysicsSolverModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		FString ShaderDir = FPaths::Combine(
			IPluginManager::Get().FindPlugin(TEXT("CustomPhysicsSolverPlugin"))->GetBaseDir(),
			TEXT("Shaders"));
		AddShaderSourceDirectoryMapping(TEXT("/Plugin/CustomPhysicsSolver"), ShaderDir);
	}
};

IMPLEMENT_MODULE(FCustomPhysicsSolverModule, CustomPhysicsSolver)
