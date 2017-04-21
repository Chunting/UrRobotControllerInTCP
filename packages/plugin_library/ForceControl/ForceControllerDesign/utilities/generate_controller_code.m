function generate_controller_code()
%generate_controller_code
%

%   Copyright 2012 The MathWorks, Inc.

% Use the Simulink Coder API to generate code for controllerModel:

controllerModel = 'ForceController';

if(~bdIsLoaded(controllerModel))
    open_system(controllerModel);
end

slbuild(controllerModel);
coder.report.generate(controllerModel);

project = simulinkproject;
projectRoot = project.RootFolder;
srcPath=[projectRoot '/work/ForceController_ert_rtw'];
cd(projectRoot);
endPath='../../ParameterTunner';
%copyfile([srcPath '/ert_main.cpp'],endPath);
copyfile([srcPath '/ForceController.cpp'],endPath);
copyfile([srcPath '/ForceController.h'],endPath);
copyfile([srcPath '/ForceController_private.h'],endPath);
copyfile([srcPath '/ForceController_types.h'],endPath);
copyfile([srcPath '/rtwtypes.h'],endPath);

cd(projectRoot);
endPath='../../SimpleUi/DragApp/';
%copyfile([srcPath '/ert_main.cpp'],endPath);
copyfile([srcPath '/ForceController.cpp'],endPath);
copyfile([srcPath '/ForceController.h'],endPath);
copyfile([srcPath '/ForceController_private.h'],endPath);
copyfile([srcPath '/ForceController_types.h'],endPath);
copyfile([srcPath '/rtwtypes.h'],endPath);

end

