#source file search paths
VPATH = ./src ../raven-set ../autocell/src

	# compiler include search paths
INCS = -I./include \
	-I../windex/include \
	-I../raven-set \
	-I../autocell/src \
	-I../../code_ext/boost/boost1_78 

LIBS = \
	-lgdiplus -lgdi32 -lcomdlg32 -lComctl32 \
		-L../../code_ext/boost/boost1_78/lib \
		-lboost_system-mgw11-mt-x64-1_78 \
		-lboost_program_options-mgw11-mt-x64-1_78 \
	#-mwindows

static_link_flags = -static-libstdc++ -static-libgcc -static

# folders
ODIR = ./.vscode/obj

OPT = -g
CCflags = -std=c++17 $(OPT) -D_mingw_

# sources
OBJ_starter = \
	$(ODIR)/main.o \
	$(ODIR)/cGraph.o

OBJ_obstacle = \
	$(ODIR)/mainObstacle.o \
	$(ODIR)/cObstacle.o \
	$(ODIR)/cObstacleGUI.o \
	$(ODIR)/cGraph.o \
	$(ODIR)/autocell.o \
	$(ODIR)/cxy.o \
	$(ODIR)/cRunWatch.o

OBJ_test = \
	$(ODIR)/test.o \
	$(ODIR)/cGraph.o \
	$(ODIR)/cutest.o

OBJ_testObstacle = \
	$(ODIR)/testObstacle.o \
	$(ODIR)/cObstacle.o \
	$(ODIR)/cGraph.o \
		$(ODIR)/autocell.o \
	$(ODIR)/cxy.o \
		$(ODIR)/cRunWatch.o \
	$(ODIR)/cutest.o

### COMPILE

$(ODIR)/%.o: %.cpp cGraph.h
	g++ -c $<  \
	-o $@ $(INCS) $(CCflags)

starter: $(OBJ_starter)
	g++ $(OPT) \
	-o bin/starter.exe $^  \
	$(static_link_flags) $(LIBS)

test: $(OBJ_test)
	g++ $(OPT) \
	-o bin/test.exe $^  \
	$(static_link_flags) $(LIBS)

testObstacle : $(OBJ_testObstacle)
	g++ -o bin/testObstacle.exe \
	$(OPT) $^ $(static_link_flags) $(LIBS)

obstacle: $(OBJ_obstacle)
	g++ -o bin/obstacle.exe \
	$^ $(OPT) \
	$(static_link_flags) $(LIBS)



.PHONY: clean
clean:
	del /q .vscode\\obj\\*.o

build: starter test obstacle

rebuild: clean starter test obstacle

