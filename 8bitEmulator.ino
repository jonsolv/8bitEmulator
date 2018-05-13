/*

*/
#define BUS_DATA 2
#define CLK 3
#define BUS_LATCH 4

#define HT  0b1000000000000000  // Halt clock
#define MI  0b0100000000000000  // Memory address register in
#define RI  0b0010000000000000  // RAM data in
#define RO  0b0001000000000000  // RAM data out
#define IO  0b0000100000000000  // Instruction register out
#define II  0b0000010000000000  // Instruction register in
#define AI  0b0000001000000000  // A register in
#define AO  0b0000000100000000  // A register out
#define EO  0b0000000010000000  // ALU out
#define SU  0b0000000001000000  // ALU subtract
#define BI  0b0000000000100000  // B register in
#define OI  0b0000000000010000  // Output register in
#define CE  0b0000000000001000  // Program counter enable
#define CO  0b0000000000000100  // Program counter out
#define JP  0b0000000000000010 // Jump (program counter in)

const uint16_t data[] = {
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 0000 - NOP
  MI|CO,  RO|II|CE,  IO|MI,  RO|AI,  0,         0, 0, 0,   // 0001 - LDA
  MI|CO,  RO|II|CE,  IO|MI,  RO|BI,  EO|AI,     0, 0, 0,   // 0010 - ADD
  MI|CO,  RO|II|CE,  IO|MI,  RO|BI,  EO|AI|SU,  0, 0, 0,   // 0011 - SUB
  MI|CO,  RO|II|CE,  IO|MI,  AO|RI,  0,         0, 0, 0,   // 0100 - STA
  MI|CO,  RO|II|CE,  IO|AI,  0,      0,         0, 0, 0,   // 0101 - LDI
  MI|CO,  RO|II|CE,  IO|JP,  0,      0,         0, 0, 0,   // 0110 - JMP
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 0111
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1000
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1001
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1010
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1011
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1100
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0, 0,   // 1101
  MI|CO,  RO|II|CE,  AO|OI,  0,      0,         0, 0, 0,   // 1110 - OUT
  MI|CO,  RO|II|CE,  HT,     0,      0,         0, 0, 0,   // 1111 - HLT
};

    int a; //A Reg
    int b; //B Reg
    int i; //Instruction Reg
    int bus; //BUS value
    int aluSum; //ALU result
    int pc; //program counter
    int mcp; //Microcode pointer
    int addressRAM; //RAM address
    int mcEepromAddr; //Address to read from Eeeprom for MC
    int runme;

    int memory[16]; //RAM

    bool hlt; //Halt if set
    bool rst; //reset all registers
    bool busShow; //output to bus set bit
    bool subtract; //ALU subtract set bit
    bool output; //Output to serial
    bool readA, writeA; //ReadWriteEnable RegA
    bool readB, writeB; //ReadWriteEnable RegB
    bool readAlu; //Read ALU to bus
    //bool readRAM, writeRAM; //ReadWrite RAM


void setup() {
  pinMode (BUS_DATA, OUTPUT);
  pinMode (CLK, OUTPUT);
  pinMode (BUS_LATCH, OUTPUT);
  Serial.begin(57600);
  reset(1); //reset all registers
  coder(); //read program code
}

void halt(int enableHalt){ //HT
  if(enableHalt == 1){
    //Serial.println("Halted");
    for(int r=0;r=-1;r+=0){
      delay(10000);
    };
  }
}

void readRegA(bool readEnable){ // AO
  if (readEnable == 1) { bus = a; }
}

void writeRegA(bool writeEnable){ // AI
  if (writeEnable == 1) { a = bus; }
}

void writeRegB(bool writeEnable){ // BI
  if (writeEnable == 1) { b = bus; }
}

void alu(bool readEnable, bool subtract){ // EO SU
  //add A & B registers unless substract bit set, enable to output to bus
  aluSum = (subtract == 1) ? aluSum = a - b : aluSum = a + b;
  if (readEnable == 1) { bus = aluSum; }
  if (subtract == 1) { }
}

void pCounterout(bool readEnable){ //CO
  if(readEnable == 1){ bus = pc; }
}

void RAMAddress(bool writeEnable){ //MI
  if(writeEnable == 1){ addressRAM = bus; }
}

void readRAM(bool readEnable){ //RO
  if (readEnable == 1) { bus = memory[addressRAM]; }
}

void writeRAM(bool writeEnable){ //RI
  if (writeEnable == 1) { memory[addressRAM] = bus;     //Serial.print("RI=1 ");
  }
}

void jmpPcounter(bool jmp){ //JP
  if (jmp == 1) { pc = bus; }
}

void incPcounter(bool inc){ //CE
  if(inc == 1) { pc +=1; }
}

void showBus() { // Show Register
  //shiftout bus to pin 2 and latch pin 4
  digitalWrite(BUS_LATCH, LOW);
  shiftOut(BUS_DATA, CLK, LSBFIRST, bus);
  digitalWrite(BUS_LATCH, HIGH);
  digitalWrite(BUS_LATCH, LOW);
}

void reset(int rst){ // RESET ALL
  if (rst == 1) {
    a, b, bus, aluSum, pc, mcp = 0;
    hlt, busShow, subtract, rst, output = 0;
  }
}

void outputBus(int enableOutput) { // OI
  if (enableOutput == 1) {
  digitalWrite(BUS_LATCH, LOW);
  shiftOut(BUS_DATA, CLK, LSBFIRST, bus);
  digitalWrite(BUS_LATCH, HIGH);
  digitalWrite(BUS_LATCH, LOW);
  }
}

void dump() { //dump
  String aa = "A=";
  String bb = " : B=";
  String busb = " : bus=";
  String ii = " : I=";
  String gy = " : EEPROM=";
  String pcc = " : pCounter=";
  String i4 = " : Instruction=";
  String i5 = String(i >> 4);
  //Serial.println(aa);
  Serial.println(aa + a + bb + b + busb + bus + ii + i + gy + runme + pcc + pc + i4 + i5);
  //print_binary(mcEepromAddr,7);
  Serial.print(" = ");
  //print_binary(data[mcEepromAddr],16);
  Serial.println();
  Serial.println();
}


void readInstructRegister(bool readEnable){ // IO
  if(readEnable == 1){ bus = (i & B00001111); }
}

void writeInstructRegister(bool writeEnable){ // II
  if(writeEnable == 1){ i = bus; }
}

int getInstruction(int iii){
  return (iii >> 4);
}

int getMCaddress(int iii){
  mcEepromAddr = ((getInstruction(iii)) << 3) + mcp;
  return mcEepromAddr;
}

void coder(){
  memory[0] = 0x1f;
  memory[1] = B00101110;
  memory[2] = B00111101;
  memory[3] = B11100000;
  memory[4] = B11110000;
  memory[13] = 2;
  memory[14] = 8;
  memory[15] = 100;
}

void runner(){
  halt(bitRead(data[mcEepromAddr],15));
  pCounterout(bitRead(data[mcEepromAddr],2));
  readInstructRegister(bitRead(data[mcEepromAddr],11));
  readRAM(bitRead(data[mcEepromAddr],12));
  readRegA(bitRead(data[mcEepromAddr],8));

  alu((bitRead(data[mcEepromAddr],7)),(bitRead(data[mcEepromAddr],6)));

  RAMAddress(bitRead(data[mcEepromAddr],14));
  outputBus(bitRead(data[mcEepromAddr],4));
  writeInstructRegister(bitRead(data[mcEepromAddr],10));
  writeRegA(bitRead(data[mcEepromAddr],9));
  writeRegB(bitRead(data[mcEepromAddr],5));
  writeRAM(bitRead(data[mcEepromAddr],13));

  jmpPcounter(bitRead(data[mcEepromAddr],1));
  incPcounter(bitRead(data[mcEepromAddr],3));
  //Serial.println();
}

void loop() {

  runme = data[getMCaddress(i)];
  runner();
  if (mcp < 8) { mcp++; } else { mcp = 0; };
  //dump();
  //showBus();
  //delay(2000);

}
