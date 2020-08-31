/*
 *  preference.h
 *  PStateChanger
 *
 */

typedef struct {
	int interval;	// ms
	int pstate_min;	// 0..
	int pstate_max;	//
	int threshold_highest;	// %
	int threshold_up;	// %
	int threshold_down;	// %
	int threshold_lowest;	// %
	int doc_method;
} PSTATECHANGER_PREF;

int checkPref(PSTATECHANGER_PREF*, int pstateCount, int minInterval);
int savePref(PSTATECHANGER_PREF*);
void loadPref(PSTATECHANGER_PREF*, int);
void defaultPref(PSTATECHANGER_PREF*, int);
