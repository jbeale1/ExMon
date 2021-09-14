// Processing code (basically Java) to read csv file
// and display one column as set of shaded rectangles
// 13-Sep-2021 JPB

BufferedReader reader;
String line;

// String fname = "C:\\Users\\beale\\Documents\\LoRa\\2021-09-13-walk2.csv";
//String fname = "C:\\Users\\beale\\Documents\\LoRa\\2021-09-13-test.csv";
String fname = "C:\\Users\\beale\\Documents\\LoRa\\2021-09-13-walk3.csv";

int xsz=1200;  // dimensions of output window
int ysz=180;

void setup() {
  size(1200,180);  // output graphics display window
  reader = createReader(fname);
  frameRate(2000);  // rate that draw() loop executes
  
}

int row[] = {};  // one row of activity data from file
int sx = 4;  // x size of rectangles
int sy = 10;  // y size of rectangles
int lineNum = 0;  // current file line number

void draw() {
  try {
    line = reader.readLine();
  } catch (IOException e) {
    e.printStackTrace();
    line = null;
  }
  if (line == null) { // EOF or error
    noLoop();
  } else {
    String[] pieces = split(line, ','); 
    if (lineNum != 0) {  // ignore header which is first line, #0
      // println(lineNum, pieces[4], pieces[5]); // [4] is the motion array
      String ar = pieces[4];
      print(ar," , ");
      int len = ar.length(); // count of characters in string
      for (int i =0; i<len; i++) {
        String ss = ar.substring(i,i+1);  // individual hex char in string
        int v =  17*unhex(ss);  // ranges from 0 to 255
        print(v," ");
        fill(v);
        stroke(v);
        int xc = 10 + lineNum*sx;
        int yc = ysz - (25 + i*sy);
        rect( xc,yc, sx,sy );
      }
      println();
    }
    lineNum++;
  }

}  // end main display loop
