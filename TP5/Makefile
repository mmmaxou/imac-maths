CC	= g++
CFLAGS	= -O2 -Wall -std=c++11 -I /usr/include/eigen3
LDFLAGS	= 
EXEC	= main
SRC	= $(wildcard *.cpp)
OBJ	= $(SRC:.cpp=.o)

all: $(EXEC)

main: $(OBJ)
	@$(CC) -o $@ $^ $(LDFLAGS)
	@echo "-------------------------------"
	@echo "    to execute type: ./main    "
	@echo "-------------------------------"

main.o :

%.o: %.cpp
	@echo "compile $@"
	@$(CC) -o $@ -c $< $(CFLAGS)
	@echo "done..."

.PHONY: clean mrproper

clean :	
	@echo "**************************"
	@echo "CLEAN"
	@echo "**************************"
	$(RM) *~ $(OBJ) $(EXEC) 

tar : clean 
	@echo "**************************"
	@echo "TAR"
	@echo "**************************"
	cd .. && tar cvfz $(BACKUP) $(DIRNAME)