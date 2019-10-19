#! /usr/bin/env python


def main():
	# Second level motor {F6,F7}
	# First level motor  {F4,F5}
	# ground motor       {F0,F1,F2}

	positionMap = { \
	'C1': ['F0','F5','F7'], 'C2':['F1','F5','F7'], 'C3':['F2','F5','F7'], \
	'B1': ['F0','F5','F6'], 'B2':['F1','F5','F6'], 'B3':['F2','F5','F6'],  \
	'A1': ['F0','F4','F6'], 'A2':['F1','F4','F6'], 'A3':['F2','F4','F6'],  \
	}


if __name__== "__main__":
  main()