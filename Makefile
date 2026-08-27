#----------------------------------------------------------------------------------
# 1. 工具链配置 
#----------------------------------------------------------------------------------
# 交叉编译器路径 (根据参考项目配置)
CC = /home/zyl/3rdparty/echiev/arm64/gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc
CPP = /home/zyl/3rdparty/echiev/arm64/gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++

#----------------------------------------------------------------------------------
# 2. 目录与路径配置 (结合 CMakeLists.txt 与 参考项目)
#----------------------------------------------------------------------------------
ROOT = .
ROOT_PARENT = ../


THIRDPARTY = /home/zyl/3rdparty/echiev/arm64/3rdparty
ROOTFS_DIR = /home/zyl/3rdparty/echiev/arm64/rfs

# 目标文件名
TARGET := low_detection_arm

# 输出目录 (可选，这里直接输出在根目录)
OBJDIR = $(ROOT)


#----------------------------------------------------------------------------------
# 3. 源文件管理 (提取自 CMakeLists.txt)
#----------------------------------------------------------------------------------
# 根据 CMakeLists.txt 中的 AUX_SOURCE_DIRECTORY 定义
# 仅纳入当前低矮检测项目真实需要的 mapfilter 子集，避免把项目 A 专用的
# get_roi / hdmap_roi_filter 等 legacy 文件一并编译进来。
SRC_DIRS := src ls_driver/Include common include
MAPFILTER_SRC := \
    mapfilter/read_hdmap.cpp \
    mapfilter/convert.cpp \
    mapfilter/hdmap_manager.cpp \
    mapfilter/coordinate_transformer.cpp \
    mapfilter/localization_manager.cpp \
    mapfilter/hdmap_filter.cpp \
    mapfilter/pcap_localization_feed.cpp

# 查找所有 C++ 和 C 源文件
CPP_SRCS := $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.cpp)) $(MAPFILTER_SRC)
C_SRCS   := $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.c))

# 将源文件映射为目标文件 (.o)
OBJS := $(CPP_SRCS:.cpp=.o) $(C_SRCS:.c=.o)

# 依赖文件
DEP_FILE := hdr_deps


#----------------------------------------------------------------------------------
# 4. 编译选项与宏定义
#----------------------------------------------------------------------------------
# 基础编译选项 (参考 CC dev.txt，保留 -fopenmp, -O3, -g（这个加上会导致程序是Debug）)
CXXFLAGS := -std=c++14 -O3 -Wall -fopenmp
CFLAGS   := -O3 -Wall -fopenmp

# 速腾驱动有使用pcap，但是ARM平台没有pcap的库
CXXFLAGS += -DNO_USE_PCAP
# ARM平台用的ulog地址
CXXFLAGS += -DLOG_CFG_FILE_PATH=\"/etc/echiev/low_detection/ulog.cfg\"

#----------------------------------------------------------------------------------
# 5. 头文件包含路径 (-I)
# 结合 CMakeLists.txt 的 include_directories 和参考项目的结构
# 第三方库头文件 (根据 CMakeLists.txt 路径推断)
#----------------------------------------------------------------------------------
INCLUDES := \
    -I$(ROOT)/include \
    -I$(ROOT)/common \
    -I$(ROOT)/mapfilter \
    -I$(ROOT)/ls_driver/Include \
    -I$(ROOT)/driver \
    -I$(THIRDPARTY)/comm/comm_2.7.1/include \
    -I$(THIRDPARTY)/libconfig/include \
        -I$(THIRDPARTY)/ulog-old/include \
    -I$(THIRDPARTY)/eigen/include/eigen3 \
    -I$(THIRDPARTY)/opencv-hw/opencv/include \
    -I$(THIRDPARTY)/opencv-hw/opencv/include/opencv \
    -I$(THIRDPARTY)/opencv-hw/opencv/include/opencv2 \
    -I$(THIRDPARTY)/pcl/include/pcl-1.8 \
    -I$(THIRDPARTY)/vtk/include/vtk-6.3 \
    -I$(THIRDPARTY)/boost/include \
    -I$(THIRDPARTY)/flann/include


#----------------------------------------------------------------------------------
# 6. 库文件链接配置 (-L -l)
# 提取自 CMakeLists.txt 中注释掉的库路径和参考项目的链接库
#----------------------------------------------------------------------------------

# 库文件搜索路径
LDFLAGS := \
    -L$(THIRDPARTY)/comm/comm_2.7.1/lib \
    -L$(THIRDPARTY)/libconfig/lib \
     -L$(THIRDPARTY)/ulog-old/lib \
    -L$(THIRDPARTY)/opencv-hw/opencv/lib \
    -L$(THIRDPARTY)/pcl/lib \
    -L$(THIRDPARTY)/vtk/lib \
    -L$(THIRDPARTY)/boost/lib \
    -L$(THIRDPARTY)/flann/lib \
    -L$(ROOTFS_DIR)/usr/lib \
    -L$(ROOTFS_DIR)/usr/lib/aarch64-linux-gnu \
    -L$(ROOTFS_DIR)/lib

# 链接库列表
# 顺序很重要：先链接高层库，再链接底层依赖
# 项目自定义库
# OpenCV (根据 CMakeLists 中的 find_package(OpenCV)) \
# PCL (根据 CMakeLists 中的 find_package(PCL ...)) \
# VTK (PCL 可视化依赖) \
# 系统基础库 (参考 感知Makefile 尾部) \

ifndef NO_USE_PCAP
LIBS += -lpcap
endif

LIBS := \
    -lcomm \
    -lconfig \
    -lconfig++ \
    -lulog \
    \
    -lboost_atomic     -lboost_chrono     -lboost_container -lboost_context \
    -lboost_coroutine  -lboost_date_time  -lboost_fiber     -lboost_filesystem \
    -lboost_graph      -lboost_iostreams  -lboost_locale    -lboost_log \
    -lboost_log_setup  -lboost_math_c99   -lboost_math_c99f -lboost_math_c99l \
    -lboost_math_tr1   -lboost_math_tr1f  -lboost_math_tr1l -lboost_prg_exec_monitor \
    -lboost_signals    -lboost_system     -lboost_thread    -lboost_timer \
    -lboost_program_options -lboost_random -lboost_regex -lboost_serialization \
    -lboost_type_erasure -lboost_unit_test_framework -lboost_wave -lboost_wserialization \
    \
    -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_video \
    \
    -lpcl_2d               -lpcl_io_ply   -lpcl_recognition  -lpcl_registration \
    -lpcl_sample_consensus -lpcl_search   -lpcl_stereo       -lpcl_surface \
    -lpcl_tracking         -lpcl_common   -lpcl_io           -lpcl_visualization \
    -lpcl_segmentation     -lpcl_surface  -lpcl_filters      -lpcl_kdtree \
    -lpcl_features         -lpcl_octree   -lpcl_ml           -lpcl_outofcore \
    \
    -lvtkalglib-6.3                -lvtkChartsCore-6.3       -lvtkCommonColor-6.3      -lvtkCommonComputationalGeometry-6.3 -lvtkCommonCore-6.3         -lvtkCommonDataModel-6.3 \
    -lvtkCommonExecutionModel-6.3 -lvtkCommonMath-6.3      -lvtkCommonMisc-6.3       -lvtkCommonSystem-6.3             -lvtkCommonTransforms-6.3    -lvtkDICOMParser-6.3 \
    -lvtkDomainsChemistry-6.3      -lvtkexoIIc-6.3           -lvtkexpat-6.3            -lvtkFiltersAMR-6.3               -lvtkFiltersCore-6.3        -lvtkFiltersExtraction-6.3 \
    -lvtkFiltersFlowPaths-6.3      -lvtkFiltersGeneral-6.3    -lvtkFiltersGeneric-6.3    -lvtkFiltersGeometry-6.3           -lvtkFiltersHybrid-6.3       -lvtkFiltersHyperTree-6.3 \
    -lvtkFiltersImaging-6.3        -lvtkFiltersModeling-6.3   -lvtkFiltersParallel-6.3   -lvtkFiltersParallelImaging-6.3   -lvtkFiltersProgrammable-6.3 -lvtkFiltersSelection-6.3 \
    -lvtkFiltersSMP-6.3            -lvtkFiltersSources-6.3    -lvtkFiltersStatistics-6.3 -lvtkFiltersTexture-6.3           -lvtkFiltersVerdict-6.3     -lvtkfreetype-6.3 \
    -lvtkftgl-6.3               -lvtkGeovisCore-6.3       -lvtkgl2ps-6.3            -lvtkhdf5-6.3                     -lvtkhdf5_hl-6.3            -lvtkImagingColor-6.3 \
    -lvtkImagingCore-6.3          -lvtkImagingFourier-6.3   -lvtkImagingGeneral-6.3    -lvtkImagingHybrid-6.3            -lvtkImagingMath-6.3         -lvtkImagingMorphological-6.3 \
    -lvtkImagingSources-6.3       -lvtkImagingStatistics-6.3 -lvtkImagingStencil-6.3   -lvtkInfovisCore-6.3              -lvtkInfovisLayout-6.3       -lvtkInteractionImage-6.3 \
    -lvtkInteractionStyle-6.3     -lvtkInteractionWidgets-6.3 -lvtkIOAMR-6.3            -lvtkIOCore-6.3                   -lvtkIOEnSight-6.3          -lvtkIOExodus-6.3 \
    -lvtkIOExport-6.3             -lvtkIOGeometry-6.3        -lvtkIOImage-6.3           -lvtkIOImport-6.3                 -lvtkIOInfovis-6.3          -lvtkIOLegacy-6.3 \
    -lvtkIOLSDyna-6.3             -lvtkIOMINC-6.3            -lvtkIOMovie-6.3           -lvtkIONetCDF-6.3                 -lvtkIOParallel-6.3         -lvtkIOParallelXML-6.3 \
    -lvtkIOPLY-6.3                -lvtkIOSQL-6.3             -lvtkIOVideo-6.3           -lvtkIOXML-6.3                    -lvtkIOXMLParser-6.3        -lvtkjpeg-6.3 \
    -lvtkjsoncpp-6.3              -lvtklibxml2-6.3           -lvtkmetaio-6.3            -lvtkNetCDF-6.3                   -lvtkNetCDF_cxx-6.3         -lvtkoggtheora-6.3 \
    -lvtkParallelCore-6.3         -lvtkpng-6.3               -lvtkproj4-6.3             -lvtkRenderingAnnotation-6.3      -lvtkRenderingContext2D-6.3 -lvtkRenderingContextOpenGL-6.3 \
    -lvtkRenderingCore-6.3        -lvtkRenderingFreeType-6.3 -lvtkRenderingGL2PS-6.3    -lvtkRenderingImage-6.3           -lvtkRenderingLabel-6.3      -lvtkRenderingLIC-6.3 \
    -lvtkRenderingLOD-6.3         -lvtkRenderingOpenGL-6.3   -lvtkRenderingVolume-6.3   -lvtkRenderingVolumeOpenGL-6.3    -lvtksqlite-6.3             -lvtksys-6.3 \
    -lvtktiff-6.3                 -lvtkverdict-6.3           -lvtkViewsContext2D-6.3    -lvtkViewsCore-6.3                -lvtkViewsInfovis-6.3       -lvtkzlib-6.3 \
    -lpthread \
    -lflann_cpp -lflann \
    -lrt \
    -lz\
    -ldl \
    -lm \
    -lstdc++ \
    -lgomp \
    -lX11 \
    -lX11-xcb \
    -lgpg-error -lkrb5 -lk5crypto -lcom_err -lkrb5support -lkeyutils -licui18n -licuuc -licudata -lgcrypt  -lgssapi_krb5 -lmpg123 -lvorbisfile -lxml2 -lp11-kit -lidn -ltasn1 -lnettle -lhogweed -lgmp -lgraphite2 -lblkid -ludev -lsoxr -lnuma -logg \
    -lshine -lopus -lopenjp2 -lmp3lame -lgsm -llzma -lssh-gcrypt -lopenmpt -lgme -lbluray -lgnutls -lchromaprint -lbz2 -lvdpau -lva-drm -lva-x11 -ljbig -lxcb-render -lharfbuzz -lselinux -lmount -lraw1394 -lusb-1.0 -lswresample -lva -lzvbi -lxvidcore \
    -lx265 -lx264 -lwebpmux -lwavpack -lvpx -lvorbisenc -lvorbis -ltwolame -ltheoraenc -ltheoradec -lspeex -lsnappy -lxcb-shm -lpixman-1 -ldatrie -lglib -lffi -lgthread -lthai -lcairo -lfontconfig -lpangoft2-1.0 -lfreetype -lgdk_pixbuf-2.0 -lgio-2.0 \
    -lXrender -lXinerama -lXi -lXrandr -lXcursor -lXcomposite -latk-1.0 -ldc1394 -lavcodec -lavformat -lavutil -lswscale  -ljpeg -lwebp -lpng16 -ltiff -lgmodule-2.0 -lgobject -lpango -lpangocairo -lgdk-x11     -lGL -ldrm -lexpat -lglapi -lICE \
    -lSM  -lxcb-dri2 -lxcb-dri3 -lxcb-glx -lxcb-present -lxcb-sync -lXdamage -lXext -lXfixes -lxshmfence -lXt -lXxf86vm -lxcb -lbsd -luuid -lXau -lXdmcp -lgtk-x11 -lpcre \


#----------------------------------------------------------------------------------
# 7. 构建规则
#----------------------------------------------------------------------------------

.PHONY: all dep clean

# 默认目标
all: $(TARGET)
	@echo "Build complete: $(TARGET)"

# 链接阶段
$(TARGET): $(OBJS)
	@echo "Linking $@..."
	$(CPP) $(OBJS) -o $@ $(LDFLAGS) $(LIBS)
	# 如果需要去除符号表以减小体积，取消下一行注释
    $(STRIP) $@

# C++ 编译规则
%.o: %.cpp
	@echo "Compiling C++ $<..."
	$(CPP) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# C 编译规则
%.o: %.c
	@echo "Compiling C $<..."
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# 生成依赖文件 (make dep)
# 使用 -MM 生成头文件依赖，忽略系统头文件
dep:
	@echo "Generating dependencies..."
	@rm -f $(DEP_FILE)
ifneq ($(strip $(CPP_SRCS)),)
	@$(CPP) $(CXXFLAGS) $(INCLUDES) -MM $(CPP_SRCS) >> $(DEP_FILE)
endif
ifneq ($(strip $(C_SRCS)),)
	@$(CC) $(CFLAGS) $(INCLUDES) -MM $(C_SRCS) >> $(DEP_FILE)
endif
	@echo "Dependencies generated in $(DEP_FILE)"   

# 清理
clean:
	@echo "Cleaning..."
	@rm -f $(OBJS) $(TARGET) $(DEP_FILE)

# 命令行输入： make rebuild ，就会执行重新生成 .o 文件
rebuild: clean all

# 包含自动生成的依赖文件
# 如果 hdr_deps 存在，则包含它，这样头文件修改后会触发重新编译
ifneq ($(wildcard $(DEP_FILE)),)
include $(DEP_FILE)
endif