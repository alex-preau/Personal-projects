#include <stdlib.h>
#include <stdio.h>
//the simulated registers are a 2d array, 64 bits (8 char) x256. This returns a pointer to $0
void** construct_registers_gp(){
  void **gp_registers = malloc(sizeof(void*)*256);
  for(int i; i < 256; ++i){
    gp_registers[i] = malloc(8);
   }
   //const void* pointer = gp_registers[0];
   return gp_registers;
}
void** construct_registers_special(){
  void **special_registers = malloc(sizeof(void*)*32);
  for(int i; i < 32; ++i){
    special_registers[i] = malloc(8);
   }
   //const void* pointer = gp_registers[0];
   return special_registers;
}

//this takes an array of registers, what is to be written, and the location
//the meathods are different for ints and doubles because doubles need more casting,
//compiler is unappy to use a double as a pointer
//the loc already needs to be converted
void write_to_register_long(void** registers,long contents,short loc){
  void *ptr = registers[loc];
  registers[loc]= (void**)contents;
  //printf("%d\n",contents);
  printf("%d\n",(long)(registers[loc]));
  return;
}

void write_to_register_double(void** registers,double contents,short loc){
  void *ptr = registers[loc];
  double*all_regs = (double*)registers;
  all_regs[loc]= contents;
  //printf("%d\n",contents);
  printf("%lf\n",((double)all_regs[loc]));
  return;
}

//covnerts string($num) to int(num)
short parse_gp_registers(char * reg){
  reg = &(reg[1]);
  short ret = atoi(reg);
  printf("%d\n",ret );
  return ret;

}

long read_long(void** registers, short loc){
  void *ptr = registers[loc];
  long contents =(long)ptr;
  printf("%ld\n",contents );
  return contents;
}

double read_double(void** registers, short loc){
  double * all_regs = (double*)registers;
  double contents = all_regs[loc];
  //double *contents =(double)ptr;
  printf("%lf\n",contents );
  return contents;
}
//type is 0 if int, 1 if double
void *add_gp(short first, short second,short loc,short type){
  void*ret=NULL
  if(type == 0){
    long adding = read_long(gp_registers,first)
    adding += read_long(gp_registers,second)
    ret=write_to_register_long(gp_registers,adding,loc)
  }else{
    long adding = read_double(gp_registers,first)
    adding += read_double(gp_registers,second)
    ret=write_to_register_double(gp_registers,adding,loc)
  }
  return ret
}


/*So I somehow have to take all of the commands into memory because of jumps,
If i execute it one by one and get to a jump I'll have major issues. When I jump
I'll have ot have some way of keeping track of the position I am jumpint to*/
int main(int argc, char const *argv[]) {
  void **gp_registers = construct_registers_gp();
  void **special_registers = construct_registers_special();
  //char *reg = 0;
  double reg = 14.1;
  //printf("%f\n",reg );
  write_to_register_long(gp_registers,19293,2);
  write_to_register_double(gp_registers,14.1,3);
  read_long(gp_registers,2);
  read_double(gp_registers,3);
  char *t_reg = "$10";
  //parse_gp_registers(t_reg);
  return 0;
}
