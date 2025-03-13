CC = gcc
CFLAGS = -O3 -Wall -I/usr/include/eigen3 -I. -I/usr/include/libxml2 -Wextra -I/usr/include/GL
LDFLAGS = $(shell xml2-config --libs) -lm -lX11 -lGL -lGLU -lglut -lglut -lassimp

# Create required directories
$(shell mkdir -p obj lib)

# Source files
URDF_SRC = urdf_import.c
VEGAS_SRC = vegasik.c
STL_SRC = draw_stl.c
MATH_SRC = math3d.c
TEST_SRC = iktest.c
STLVIEW_SRC = stl_viewer.c
CANVAS_SRC = canvas.c
URDFVIEW_SRC = urdf_viewer.c
URDFLOAD_SRC = urdf_loader.c
DAE_SRC = dae_viewer.c
DRAW_DAE_SRC = draw_dae.c

# Object files (in obj directory)
URDF_OBJ = obj/urdf_import.o
VEGAS_OBJ = obj/vegasik.o
STL_OBJ = obj/draw_stl.o
MATH_OBJ = obj/math3d.o
TEST_OBJ = obj/iktest.o
STLVIEW_OBJ = obj/stl_viewer.o
CANVAS_OBJ = obj/canvas.o
URDFVIEW_OBJ = obj/urdf_viewer.o
URDFLOAD_OBJ = obj/urdf_loader.o
DAE_OBJ = obj/dae_viewer.o
DRAW_DAE_OBJ = obj/draw_dae.o

# Libraries (in lib directory)
URDF_LIB = lib/liburdf.a
VEGAS_LIB = lib/libvegas.a
STL_LIB = lib/libstl.a
MATH_LIB = lib/libmath3d.a
CANVAS_LIB = lib/libcanvas.a
DAE_LIB = lib/libdae.a

# Targets
TARGETS = urdf_viewer dae_viewer iktest canvas_grid imu_sim

all: $(URDF_LIB) $(VEGAS_LIB) $(STL_LIB) $(MATH_LIB) $(CANVAS_LIB) $(DAE_LIB) iktest stl_viewer urdf_viewer urdf_loader dae_viewer canvas_grid imu_sim

$(URDF_LIB): $(URDF_OBJ)
	ar rcs $@ $^

$(VEGAS_LIB): $(VEGAS_OBJ)
	ar rcs $@ $^

$(STL_LIB): $(STL_OBJ)
	ar rcs $@ $^

$(MATH_LIB): $(MATH_OBJ)
	ar rcs $@ $^

$(CANVAS_LIB): $(CANVAS_OBJ)
	ar rcs $@ $^

$(DAE_LIB): $(DRAW_DAE_OBJ)
	ar rcs $@ $^

iktest: $(TEST_OBJ) $(URDF_LIB) $(VEGAS_LIB) $(STL_LIB) $(MATH_LIB) $(CANVAS_LIB)
	$(CC) -o $@ $(TEST_OBJ) $(CANVAS_LIB) $(MATH_LIB) $(STL_LIB) $(VEGAS_LIB) $(URDF_LIB) $(LDFLAGS)

stl_viewer: $(STLVIEW_OBJ) $(STL_LIB) $(MATH_LIB) $(CANVAS_LIB)
	$(CC) -o $@ $(STLVIEW_OBJ) $(CANVAS_LIB) $(STL_LIB) $(MATH_LIB) $(LDFLAGS)

urdf_viewer: $(URDFVIEW_OBJ) $(URDF_LIB) $(STL_LIB) $(MATH_LIB) $(CANVAS_LIB)
	$(CC) -o $@ $(URDFVIEW_OBJ) $(CANVAS_LIB) $(URDF_LIB) $(STL_LIB) $(MATH_LIB) $(LDFLAGS)

urdf_loader: $(URDFLOAD_OBJ) $(URDF_LIB) $(MATH_LIB)
	$(CC) -o $@ $(URDFLOAD_OBJ) $(URDF_LIB) $(MATH_LIB) $(STL_LIB) $(LDFLAGS)

dae_viewer: $(DAE_OBJ) $(CANVAS_LIB) $(DAE_LIB)
	$(CC) -o $@ $(DAE_OBJ) $(CANVAS_LIB) $(DAE_LIB) $(LDFLAGS)

canvas_grid: obj/canvas_grid.o $(CANVAS_LIB)
	$(CC) -o $@ $^ $(LDFLAGS)

# Add build rule for imu_sim
imu_sim: obj/imu_sim.o $(CANVAS_LIB) $(MATH_LIB)
	$(CC) -o $@ $^ $(LDFLAGS)

# Pattern rule for object files
obj/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f obj/*.o lib/*.a iktest stl_viewer urdf_viewer urdf_loader dae_viewer canvas_grid
	rmdir obj lib 2>/dev/null || true