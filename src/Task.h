#pragma once

#include <iostream>

class Task
{
public:
	virtual void output(std::ostream &out) = 0;
};