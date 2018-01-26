#include "DigraphSearchItem.h"

ostream& operator<<(ostream& out, const DigraphSearchItem& item) {
	out << "{ [" << item.lKey << "," << item.rKey << ") => " << item.value << " }" << endl;
	return out;
}