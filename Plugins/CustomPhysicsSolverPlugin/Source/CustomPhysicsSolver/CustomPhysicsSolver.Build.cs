using UnrealBuildTool;

public class CustomPhysicsSolver : ModuleRules
{
	public CustomPhysicsSolver(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"RHI",
			"RenderCore",
			"Renderer",     // RenderGraphBuilder, RenderGraphUtils, GlobalShader
			"Projects",     // IPluginManager for shader path registration
		});
	}
}
