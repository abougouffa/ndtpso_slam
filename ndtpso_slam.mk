##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=ndtpso_slam
ConfigurationName      :=Debug
WorkspacePath          :=/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam
ProjectPath            :=/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=Abdelhak
Date                   :=02/02/18
CodeLitePath           :=/home/hacko/.codelite
LinkerName             :=/usr/bin/arm-none-eabi-g++
SharedObjectLinkerName :=/usr/bin/arm-none-eabi-g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="ndtpso_slam.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  -O0
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). 
IncludePCH             := 
RcIncludePath          := 
Libs                   := 
ArLibs                 :=  
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch). $(LibraryPathSwitch)Debug 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/bin/arm-none-eabi-ar rcu
CXX      := /usr/bin/arm-none-eabi-g++
CC       := /usr/bin/arm-none-eabi-gcc
CXXFLAGS :=  -g -Wall $(Preprocessors)
CFLAGS   :=   $(Preprocessors)
ASFLAGS  := 
AS       := /usr/bin/arm-none-eabi-as


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(ObjectSuffix) $(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(ObjectSuffix) $(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.cxx$(ObjectSuffix) $(IntermediateDirectory)/lib_ndtpso_slam_core.cpp$(ObjectSuffix) $(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(ObjectSuffix) $(IntermediateDirectory)/src_ndtpso_slam_node.cpp$(ObjectSuffix) $(IntermediateDirectory)/lib_ndtpso_slam_ndtcell.cpp$(ObjectSuffix) $(IntermediateDirectory)/lib_ndtpso_slam_ndtframe.cpp$(ObjectSuffix) $(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(ObjectSuffix) $(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.c$(ObjectSuffix) \
	$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.cxx$(ObjectSuffix) $(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.c$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild MakeIntermediateDirs
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

MakeIntermediateDirs:
	@test -d ./Debug || $(MakeDirCommand) ./Debug


$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(ObjectSuffix): build/default/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp $(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/build/default/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(DependSuffix): build/default/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(DependSuffix) -MM build/default/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp

$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(PreprocessSuffix): build/default/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(PreprocessSuffix) build/default/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp

$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(ObjectSuffix): build/default/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c $(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/build/default/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(DependSuffix): build/default/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(ObjectSuffix) -MF$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(DependSuffix) -MM build/default/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c

$(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(PreprocessSuffix): build/default/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/build_default_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(PreprocessSuffix) build/default/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c

$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.cxx$(ObjectSuffix): build/release/CMakeFiles/feature_tests.cxx $(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.cxx$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/build/release/CMakeFiles/feature_tests.cxx" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.cxx$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.cxx$(DependSuffix): build/release/CMakeFiles/feature_tests.cxx
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.cxx$(ObjectSuffix) -MF$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.cxx$(DependSuffix) -MM build/release/CMakeFiles/feature_tests.cxx

$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.cxx$(PreprocessSuffix): build/release/CMakeFiles/feature_tests.cxx
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.cxx$(PreprocessSuffix) build/release/CMakeFiles/feature_tests.cxx

$(IntermediateDirectory)/lib_ndtpso_slam_core.cpp$(ObjectSuffix): lib/ndtpso_slam/core.cpp $(IntermediateDirectory)/lib_ndtpso_slam_core.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/lib/ndtpso_slam/core.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/lib_ndtpso_slam_core.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/lib_ndtpso_slam_core.cpp$(DependSuffix): lib/ndtpso_slam/core.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/lib_ndtpso_slam_core.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/lib_ndtpso_slam_core.cpp$(DependSuffix) -MM lib/ndtpso_slam/core.cpp

$(IntermediateDirectory)/lib_ndtpso_slam_core.cpp$(PreprocessSuffix): lib/ndtpso_slam/core.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/lib_ndtpso_slam_core.cpp$(PreprocessSuffix) lib/ndtpso_slam/core.cpp

$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(ObjectSuffix): build/release/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c $(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/build/release/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(DependSuffix): build/release/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(ObjectSuffix) -MF$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(DependSuffix) -MM build/release/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c

$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(PreprocessSuffix): build/release/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdC_CMakeCCompilerId.c$(PreprocessSuffix) build/release/CMakeFiles/3.5.1/CompilerIdC/CMakeCCompilerId.c

$(IntermediateDirectory)/src_ndtpso_slam_node.cpp$(ObjectSuffix): src/ndtpso_slam_node.cpp $(IntermediateDirectory)/src_ndtpso_slam_node.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/src/ndtpso_slam_node.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ndtpso_slam_node.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ndtpso_slam_node.cpp$(DependSuffix): src/ndtpso_slam_node.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ndtpso_slam_node.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ndtpso_slam_node.cpp$(DependSuffix) -MM src/ndtpso_slam_node.cpp

$(IntermediateDirectory)/src_ndtpso_slam_node.cpp$(PreprocessSuffix): src/ndtpso_slam_node.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ndtpso_slam_node.cpp$(PreprocessSuffix) src/ndtpso_slam_node.cpp

$(IntermediateDirectory)/lib_ndtpso_slam_ndtcell.cpp$(ObjectSuffix): lib/ndtpso_slam/ndtcell.cpp $(IntermediateDirectory)/lib_ndtpso_slam_ndtcell.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/lib/ndtpso_slam/ndtcell.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/lib_ndtpso_slam_ndtcell.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/lib_ndtpso_slam_ndtcell.cpp$(DependSuffix): lib/ndtpso_slam/ndtcell.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/lib_ndtpso_slam_ndtcell.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/lib_ndtpso_slam_ndtcell.cpp$(DependSuffix) -MM lib/ndtpso_slam/ndtcell.cpp

$(IntermediateDirectory)/lib_ndtpso_slam_ndtcell.cpp$(PreprocessSuffix): lib/ndtpso_slam/ndtcell.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/lib_ndtpso_slam_ndtcell.cpp$(PreprocessSuffix) lib/ndtpso_slam/ndtcell.cpp

$(IntermediateDirectory)/lib_ndtpso_slam_ndtframe.cpp$(ObjectSuffix): lib/ndtpso_slam/ndtframe.cpp $(IntermediateDirectory)/lib_ndtpso_slam_ndtframe.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/lib/ndtpso_slam/ndtframe.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/lib_ndtpso_slam_ndtframe.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/lib_ndtpso_slam_ndtframe.cpp$(DependSuffix): lib/ndtpso_slam/ndtframe.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/lib_ndtpso_slam_ndtframe.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/lib_ndtpso_slam_ndtframe.cpp$(DependSuffix) -MM lib/ndtpso_slam/ndtframe.cpp

$(IntermediateDirectory)/lib_ndtpso_slam_ndtframe.cpp$(PreprocessSuffix): lib/ndtpso_slam/ndtframe.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/lib_ndtpso_slam_ndtframe.cpp$(PreprocessSuffix) lib/ndtpso_slam/ndtframe.cpp

$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(ObjectSuffix): build/release/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp $(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/build/release/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(DependSuffix): build/release/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(DependSuffix) -MM build/release/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp

$(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(PreprocessSuffix): build/release/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/build_release_CMakeFiles_3.5.1_CompilerIdCXX_CMakeCXXCompilerId.cpp$(PreprocessSuffix) build/release/CMakeFiles/3.5.1/CompilerIdCXX/CMakeCXXCompilerId.cpp

$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.c$(ObjectSuffix): build/release/CMakeFiles/feature_tests.c $(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/build/release/CMakeFiles/feature_tests.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.c$(DependSuffix): build/release/CMakeFiles/feature_tests.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.c$(ObjectSuffix) -MF$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.c$(DependSuffix) -MM build/release/CMakeFiles/feature_tests.c

$(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.c$(PreprocessSuffix): build/release/CMakeFiles/feature_tests.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/build_release_CMakeFiles_feature_tests.c$(PreprocessSuffix) build/release/CMakeFiles/feature_tests.c

$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.cxx$(ObjectSuffix): build/default/CMakeFiles/feature_tests.cxx $(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.cxx$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/build/default/CMakeFiles/feature_tests.cxx" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.cxx$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.cxx$(DependSuffix): build/default/CMakeFiles/feature_tests.cxx
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.cxx$(ObjectSuffix) -MF$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.cxx$(DependSuffix) -MM build/default/CMakeFiles/feature_tests.cxx

$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.cxx$(PreprocessSuffix): build/default/CMakeFiles/feature_tests.cxx
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.cxx$(PreprocessSuffix) build/default/CMakeFiles/feature_tests.cxx

$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.c$(ObjectSuffix): build/default/CMakeFiles/feature_tests.c $(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/run/media/hacko/3dec2fb9-c0d3-4845-8624-25cf19ba3105/home/abdelhak/CatkinWorkspace/src/ndtpso_slam/build/default/CMakeFiles/feature_tests.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.c$(DependSuffix): build/default/CMakeFiles/feature_tests.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.c$(ObjectSuffix) -MF$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.c$(DependSuffix) -MM build/default/CMakeFiles/feature_tests.c

$(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.c$(PreprocessSuffix): build/default/CMakeFiles/feature_tests.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/build_default_CMakeFiles_feature_tests.c$(PreprocessSuffix) build/default/CMakeFiles/feature_tests.c


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


