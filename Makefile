.PHONY: all clean

CC := g++-11

SRCS := $(wildcard *.cpp)
OBJS := $(SRCS:cpp=o)

CFLAGS := `pkg-config opencv4 --cflags --libs` -O2

all: main.out

# Link .o to main
main.out: $(OBJS) 
	$(CC) -o $@ $(OBJS) $(CFLAGS)
	./main.out

# Compile .cpp to .o
$(OBJS): %.o: %.cpp 
	$(CC) -c $< $(CFLAGS)

# Clean
clean:
	rm -f $(OBJS) main.out

