gcc main.c -o main.exe -O2 -s -Wall -std=c99 -Wno-missing-braces -I include/ -L lib/ -lraylib -lopengl32 -lgdi32 -lwinmm
main.exe