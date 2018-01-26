#ifndef __DIGRAPH_SEARCH_ITEM_
#define __DIGRAPH_SEARCH_ITEM_

#include <iomanip>
#include <iostream>
#include <string>

#include "NodeAndEdge.h"

using namespace std;

class DigraphSearchItem {
public:
	int lKey;
	int rKey;
	int value;  // value between left and right keys
	bool empty;

	DigraphSearchItem() {
		empty = true;
	}

	DigraphSearchItem(int _lKey, int _rKey, int _value) {
		lKey = _lKey;
		rKey = _rKey;
		value = _value;
		empty = false;
	}

	void set(int _lKey, int _rKey, int _value) {
		lKey = _lKey;
		rKey = _rKey;
		value = _value;
	}

	bool operator< (const DigraphSearchItem& item) const {
		return rKey < item.lKey;
	}

	bool operator> (const DigraphSearchItem& item) const {
		return lKey > item.rKey;
	}

	bool operator== (const DigraphSearchItem& item) const {
		if (rKey < item.lKey) return false;
		if (lKey > item.rKey) return false;
		return true;
	}

	bool operator!= (const DigraphSearchItem& item) const {
		if (rKey < item.lKey) return true;
		if (lKey > item.rKey) return true;
		return false;
	}

	DigraphSearchItem & operator= (const DigraphSearchItem& item) {
		lKey = item.lKey;
		rKey = item.rKey;
		value = item.value;
		return *this;
	}

	friend ostream& operator<<(ostream& out, const DigraphSearchItem& item);
};

class NodeItem {
public:
	Node* node;
	int lValue;
	int rValue;

	NodeItem(Node* _node, int _lValue, int _rValue) {
		node = _node;
		lValue = _lValue;
		rValue = _rValue;
	}

	bool check(NodeItem item, int& newLValue, int& newRValue) {
		if (node != item.node) return false;

		if (lValue <= newLValue && rValue >= newRValue)
			return true;

		if (lValue <= newLValue && newLValue <= rValue) 
			newLValue = rValue;

		if (lValue <= newRValue && newRValue <= rValue) 
			newRValue = lValue;

		return false;
	}
};

#endif