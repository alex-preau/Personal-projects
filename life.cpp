#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdio>

using namespace std;
const int rows = 30;
const int columns = 60;
const char LIVE = 'X'; //life cells
const char DEAD = '.'; //dead cells

//the board is a nxm board of . or X depending on if dead or alive
//make board gives all dead
vector<vector<char> > make_board(){
  vector<vector<char> > board;
  for(int i = 0; i < rows;++i){
    vector<char> row;
    for (int j = 0; j < columns; j++) {
      row.push_back(DEAD);
    }
    board.push_back(row);
  }
  return board;
}

void print_board(vector<vector<char> >board){
  for(vector<vector<char> >::iterator row=board.begin();row<board.end();++row){
    for(vector<char>::iterator cell=row->begin();cell<row->end();++cell){
      cout<<*cell;
    }
    cout << '\n';
  }
  cout << '\n';
//  cout << '\n';
//  cout << '\n';
}

int count_living(vector<vector<char> > board,int i,int j){
  int count = 0;
  if((i > 0) && (i < rows)){
    if(board[i-1][j] == LIVE){
      ++count;
    }
  }
  if((i > -1) && (i < rows-1)){
    if(board[i+1][j] == LIVE){
      ++count;
    }
  }
  if((j > 0) && (j < columns)){
    if(board[i][j-1] == LIVE){
      ++count;
    }
  }
  if((j > -1) && (j < columns-1)){
    if(board[i][j+1] == LIVE){
      ++count;
    }
  }
  if((j > -1) && (j < columns-1) &&(i > -1) && (i < rows-1)){
    if(board[i+1][j+1] == LIVE){
      ++count;
    }
  }
  if((j > 0) && (j < columns) &&(i > -1) && (i < rows-1)){
    if(board[i+1][j-1] == LIVE){
      ++count;
    }
  }
  if((j > -1) && (j < columns-1) &&(i > 0) && (i < rows)){
    if(board[i-1][j+1] == LIVE){
      ++count;
    }
  }
  if((j > 0) && (j < columns) &&(i > 0) && (i < rows)){
    if(board[i-1][j-1] == LIVE){
      ++count;
    }
  }
  //cout << count;
  return count;

}

vector<vector<char> >  step_board(vector<vector<char> > board,vector<vector<char> > new_board){
  int count = 0;
  for(int i = 0; i < rows; ++ i){
    for (int j = 0; j < columns; j++) {
      count = count_living(board,i,j);
      if(board[i][j] == LIVE){

        if(count < 2){
          new_board[i][j] = DEAD;
        }else if(count >= 4){
          new_board[i][j] = DEAD;
        }else{
          new_board[i][j] = LIVE;
          cout << "alive\n";
        }
      }else{
        if(count == 3){
          new_board[i][j] = LIVE;
        }
      }
    }
  }
  return new_board;
  //print_board(new_board);
}

vector<vector <char> > initialize(vector<vector <char> > board){
  board[5][10] = LIVE;
  board[5][11] = LIVE;
  board[5][12] = LIVE;
  board[4][12] = LIVE;
  board[3][11] = LIVE;
  return board;
}


void swap_boards(vector<vector<char> > board,vector<vector<char> > new_board){
  board = new_board;
}
void wait ( int seconds )
{
	clock_t endwait;
	endwait = clock () + seconds * CLOCKS_PER_SEC/2 ;
	while (clock() < endwait) {}
}

int main(){
  vector<vector <char> >board = make_board();
  vector<vector <char> >new_board = make_board();
  board = initialize(board);
  print_board(board);
  int done = 0;
  while(done++ < 100){
    wait(1);
  new_board = step_board(board,new_board);
  board=new_board;
  print_board(board);
}
  return 0;
}
