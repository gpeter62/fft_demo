/*
 * gameoflife. A Conway's game of life implementation for Arduino 
 * 
*/


void resetMap()
{
  randomSeed(analogRead(0));
  for(int x = 0; x < XMAX; ++x) 
    for(int y = 0; y < YMAX; ++y){ gameBoard[x][y] = random(2);  }
}

void nextGeneration() {
  int up;
  
  for(int x = 0; x < XMAX; ++x)  {
    for(int y = 0; y < YMAX; ++y)  {
      int sum = sumNeighbours(x, y);
      if(gameBoard[x][y] == 1) //If Cell is alive
      {
        if(sum < 2 || sum > 3) {//Cell dies
          newGameBoard[x][y] = 0;
        }
        else {
          newGameBoard[x][y] = 1;
        }
      }
      else {              //If Cell is dead
        if(sum == 3) {   //A new Cell is born
          newGameBoard[x][y] = 1;
        }
      }
    }
  }
  if(compareArray() == 0)  {
    gen = maxGen - 1;
  }

  for(int x=0;x<XMAX;x++) {
  for (int y=0; y<YMAX; y++)  {
    if (newGameBoard[x][y] > gameBoard[x][y]) setPixel(x,y,green);   //newborn
    if (newGameBoard[x][y] < gameBoard[x][y]) setPixel(x,y,red2);    //died
    if (newGameBoard[x][y] == gameBoard[x][y]) {                    //not changed
      if (newGameBoard[x][y] == 0) setPixel(x,y,black);               //empty
      else                         setPixel(x,y,green);               //live
    }  
  }
 }
  
  memcpy(&gameBoard, &newGameBoard, sizeof(gameBoard));
}

/* Return 0 if they are equal, else 1 */
int compareArray()
{
 for(int i = 0; i < XMAX; ++i) 
  for (int j =0; j< YMAX; j++) {
    if(newGameBoard[i][j] != gameBoard[i][j]) { return 1; }
  }
 return 0;
}

int sumNeighbours(int x, int y)
{
  int sum = 0;
  
  for (int ix = -1; ix <=1; ++ix)  {
    for (int iy = -1; iy <=1; ++iy)  {
      if(x + ix < 0 || x + ix > XMAX - 1 || y + iy < 0 || y + iy > YMAX || (ix == 0 && iy == 0))  { continue; }
      
      if (gameBoard[x + ix][y + iy] == 1) sum++;
    }
  }
  
  return sum;
}


void nextLifeStep() { 
  if(gen >= maxGen) {
    gen = 0;
    resetMap();
  }
  
  nextGeneration();
}
