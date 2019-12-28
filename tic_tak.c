#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int board[9] = {2,3,4,5,6,5,6,7,8};
// 0  1   2
// 3  4   5
// 6  7   8
char piece(int pos){
  if(board[pos] == 0){
    return 'O';
  }  if(board[pos] == 1){
      return 'X';
    }
    return ' ';
}

void write_board(){
  printf("Current board is\n");
  printf("   |   |   \n");
  printf(" %c | %c | %c \n",piece(0),piece(1),piece(2));
  printf("___|___|___\n");
  printf("   |   |   \n");
  printf(" %c | %c | %c \n",piece(3),piece(4),piece(5));
  printf("___|___|___\n");
  printf("   |   |   \n");
  printf(" %c | %c | %c \n",piece(6),piece(7),piece(8));
}

int check_done(){
  if((board[0] == board[1]) &&(board[1] == board[2])){
    printf("done\n");
    return 1;
  }
  if((board[3] == board[4]) &&(board[4] == board[5])){
    printf("done\n");
    return 1;
  }
  if((board[6] == board[8]) &&(board[8] == board[2])){
    printf("done\n");
    return 1;
  }
  if((board[0] == board[3]) &&(board[3] == board[6])){
    printf("done\n");
    return 1;
  }
  if((board[1] == board[4]) &&(board[4] == board[7])){
    printf("done");
    return 1;
  }
  if((board[2] == board[5]) &&(board[5] == board[8])){
    printf("done");
    return 1;
  }
  if((board[0] == board[4]) &&(board[4] == board[8])){
    printf("done");
    return 1;
  }
  if((board[2] == board[4]) &&(board[4] == board[6])){
    printf("done");
    return 1;
  }
  printf("not done\n");
  return 0;
}

void modify(){
  char *line = malloc(10);
  unsigned long buffer_size = 10;
  char sept = ',';
  getline(&line,&buffer_size,stdin);
  char *player = strtok(line, &sept);
  char *position = strtok(NULL, &sept);
  printf("%s",position);
  if(strcmp((player), "1")){
    board[atoi(position)] = 0;
  }else{
    board[atoi(position)] = 1;
  }
}

int main(){
  printf("Please type move in format\n");
  printf("1,3\n");
  printf("for player 1 moving position 3\n");
  printf("player 1 is X, player 2 is O\n");
  printf("the board format is\n");
  printf("   |   |   \n");
  printf(" %d | %d | %d \n",0,1,2);
  printf("___|___|___\n");
  printf("   |   |   \n");
  printf(" %d | %d | %d \n",3,4,5);
  printf("___|___|___\n");
  printf("   |   |   \n");
  printf(" %d | %d | %d \n\n",6,7,8);
  write_board();
  int done = 0;

  while(!done){
  modify();
  write_board();
  //printf("%d",done);
  done = check_done();
}
printf("Somebody won! Go you\n");
return 0;
}
