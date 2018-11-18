TARGET := x100.elf
BUILD_FOLDER := build/

CC       := gcc
CC_FLAGS := -O0 -g -Wall -std=c99 
C_DIRS   := src
INC      := include	

INC        := $(addprefix -I, $(INC))	
VPATH      := $(C_DIRS)
C_DIRS_TMP := $(addsuffix /*.c, $(C_DIRS))
C_SRC      := $(wildcard $(C_DIRS_TMP))
C_SRC      := $(notdir $(C_SRC))	
C_OBJ      := $(C_SRC:%.c=%.o)
		
.PHONY: all
all: build_dir $(TARGET)

$(C_OBJ): %.o : %.c
	@printf '>>>>>>> Compiling file: $< <<<<<<<\n'
	$(CC) $(CC_FLAGS) $(INC) -c $< -o $(BUILD_FOLDER)/"$@"
	@printf "\n"
	
$(TARGET): $(C_OBJ)
	@printf '>>>>>>> Compiling elf file: <<<<<<<\n'
	$(CC) $(addprefix $(BUILD_FOLDER), $(C_OBJ)) -o $(addprefix $(BUILD_FOLDER),$(TARGET))
	@printf '\n'	
	
.PHONY:	build_dir
build_dir:	
	mkdir -p ${BUILD_FOLDER}	
	
.PHONY: clean
clean: 
	rm -rf $(BUILD_FOLDER)*.o $(BUILD_FOLDER)*.elf	
	
.PHONY: print
print:
	@printf "VPATH: $(VPATH)"
	@printf "C_SRC: $(C_SRC) \n"
	@printf "C_OBJ: $(C_OBJ) \n"	
	@printf "\n"