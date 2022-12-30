#John Mathews
#hw04 - Optimal Path Finding
# CPSC 2120

#makefile for hw04

#configure variables
CC = g++
CFLAGS = -Wall
LDFLAGS = -lm
OBJFILES = hw04.o
TARGET = hw04

# target for compiling
all: $(TARGET)

#run program with c-l arguments
run: all
	./hw04 config.txt railway.ppm path.txt

#compile using gcc and variables specified
$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJFILES) $(LDFLAGS)

#clean file directory of object files
clean:
	rm -f makefile~ $(OBJFILES) $(TARGET)
