#include <yarp/os/all.h>
#include <iostream>

using namespace std;
using namespace yarp::os;

int main(int argc, char const *argv[])
{
	Network yarp;
	BufferedPort<Bottle> port;
	port.open("/summer");
	while(true) {
	    cout << "waiting for input" << endl;
	    Bottle* input = port.read();
	    if(input != NULL) {
	    	cout << "got " << input->toString().c_str() << endl;
	    	double total = 0;
	    	for(unsigned i = 0; i < input->size(); i++) {
	    		total += input->get(i).asDouble();
	    	}
	    	Bottle& output = port.prepare();
	    	output.clear();
	    	output.addString("total");
	    	output.addDouble(total);
	    	cout << "writting " << output.toString().c_str() << endl;
	    	port.write();
	    }
	}
	return 0;
}