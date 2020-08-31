//
//  PStateChangerAppDelegate.m
//  PStateChanger
//
//

#import "PStateChangerAppDelegate.h"
#import "LoadChart.h"
#include "monitor.h"
#include "KextIntf.h"
#include "preference.h"

enum {
	DATA_COUNT = 60,
};
static float _data[DATA_COUNT];
static float _pstate[DATA_COUNT];
static int _dataIndex;
static PSTATE_CTL_INFO _pstate_table[32];
static int _pstateCount = -1;
static PSTATECHANGER_PREF _pref;
static PSTATE_STATUS_INFO _info;
static float _cpuload;

static void init_local(){
	if(_pstateCount < 0){
		_pstateCount = getPstateTable(_pstate_table);
		getPstateCurrent(&_info);
		float ratio = (float)_info.frequency / _pstate_table[0].frequency;
		for(int k = 0; k< DATA_COUNT; k++){
			_data[k] = 0.0f;
			_pstate[k] = ratio;
		}
		loadPref(&_pref, _pstateCount);
		_dataIndex = DATA_COUNT-1;
		monitor_init();
	}
}

@implementation PStateChangerAppDelegate

@synthesize window;

#pragma mark prefForm
- (void)fillForm:(PSTATECHANGER_PREF*) pref
{
	[[prefForm cellAtIndex:0] setIntValue:pref->interval];
	[[prefForm cellAtIndex:1] setIntValue:pref->pstate_min];
	[[prefForm cellAtIndex:2] setIntValue:pref->pstate_max];
	[[prefForm cellAtIndex:3] setIntValue:pref->threshold_highest];
	[[prefForm cellAtIndex:4] setIntValue:pref->threshold_up];
	[[prefForm cellAtIndex:5] setIntValue:pref->threshold_down];
	[[prefForm cellAtIndex:6] setIntValue:pref->threshold_lowest];
	[[prefForm cellAtIndex:7] setIntValue:pref->doc_method];
}

- (void)extractForm:(PSTATECHANGER_PREF*) pref
{
	pref->interval = [[prefForm cellAtIndex:0] intValue];
	pref->pstate_min = [[prefForm cellAtIndex:1] intValue];
	pref->pstate_max = [[prefForm cellAtIndex:2] intValue];
	pref->threshold_highest = [[prefForm cellAtIndex:3] intValue];
	pref->threshold_up = [[prefForm cellAtIndex:4] intValue];
	pref->threshold_down = [[prefForm cellAtIndex:5] intValue];
	pref->threshold_lowest = [[prefForm cellAtIndex:6] intValue];
	pref->doc_method = [[prefForm cellAtIndex:7] intValue];
}

- (void)colorForm:(PSTATECHANGER_PREF*) pref
{
#if	0
	[[prefForm cellAtIndex:0] setColor:(pref == nil || pref->interval >= 0) ? [NSColor blackColor]:[NSColor redColor]];
	[[prefForm cellAtIndex:1] setColor:(pref == nil || pref->pstate_min >= 0) ? [NSColor blackColor]:[NSColor redColor]];
	[[prefForm cellAtIndex:2] setColor:(pref == nil || pref->pstate_max >= 0) ? [NSColor blackColor]:[NSColor redColor]];
	[[prefForm cellAtIndex:3] setColor:(pref == nil || pref->threshold_highest >= 0) ? [NSColor blackColor]:[NSColor redColor]];
	[[prefForm cellAtIndex:4] setColor:(pref == nil || pref->threshold_up >= 0) ? [NSColor blackColor]:[NSColor redColor]];
	[[prefForm cellAtIndex:5] setColor:(pref == nil || pref->threshold_down >= 0) ? [NSColor blackColor]:[NSColor redColor]];
	[[prefForm cellAtIndex:6] setColor:(pref == nil || pref->threshold_lowest >= 0) ? [NSColor blackColor]:[NSColor redColor]];
#endif
}


#pragma mark init
- (void)applicationDidHide:(NSNotification *)notification {
	if(_pref.doc_method)
		[[NSApp dockTile] setContentView: docview];
}

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
	// Insert code here to initialize your application
	init_local();

	// experimental
	docview = [[LoadChart alloc] init];
	[docview setFrameWidth:1.0];

	autoThrottle = [autoCheckBox state] == NSOnState;
	[self fillForm: &_pref];

	updateTimer = [[NSTimer scheduledTimerWithTimeInterval:(float)_pref.interval / 1000
													target:self
												  selector:@selector(updateInfo:)
												  userInfo:nil
												   repeats:YES] retain];
	if(![window isVisible]){
		[self applicationDidHide:nil];
	}
	if(updateTimer)
		[updateTimer fire];
}

- (void)applicationWillTerminate:(NSNotification *)aNotification {
	//
	if(updateTimer){
		[updateTimer invalidate];
		[updateTimer release];
	}
	[docview release];
}


- (void)applicationDidUnhide:(NSNotification *)notification {
	if(_pref.doc_method)
		[[NSApp dockTile] setContentView: nil];
}

- (void)updateInfo:(NSTimer *)timer {
	
	_cpuload = monitor_probe();
	getPstateCurrent(&_info);
	
	float ratio = (float)_info.frequency / _pstate_table[0].frequency;
	_pstate[_dataIndex] = ratio;
	_data[_dataIndex] = ratio * _cpuload;

	_dataIndex = (_dataIndex + 1) % DATA_COUNT;

	if([window isVisible]){
		[sview reloadData];
		
		if(lcview){	// update
			[lcview setData:_data pstate:_pstate count:DATA_COUNT start:_dataIndex];
		}
	} else {
		if(_pref.doc_method){
			[docview setData:_data pstate:_pstate count:DATA_COUNT start:_dataIndex];
			[[NSApp dockTile] display];
		}
	}
	// throttling
	uint8_t newState = _info.pstate;
	int iLoad = _cpuload * 100;
	if(autoThrottle){
		if(iLoad > _pref.threshold_highest){	// up to highest
			newState = _pref.pstate_min;
		} else if(iLoad > _pref.threshold_up){
			if(newState > _pref.pstate_min)
				newState -= 1;
		} else if(iLoad < _pref.threshold_down){
			newState += 1;
		} else if(iLoad < _pref.threshold_lowest){	// down to lowest
			newState = _pref.pstate_max;
		}
		
	} else {
		newState = fixedState;
	}
	if(newState > _pref.pstate_max)
		newState = _pref.pstate_max;
	if(newState != _info.pstate)
		setPstate(newState);
}



#pragma mark tableDelegate
- (NSInteger)numberOfRowsInTableView:(NSTableView *)aTableView
{
	init_local();
	if(aTableView == tview)
		return _pstateCount;
	if(aTableView == sview)
		return 5;
	return 0;
}

- (id)tableView:(NSTableView *)aTableView objectValueForTableColumn:(NSTableColumn *)aTableColumn row:(NSInteger)rowIndex
{
	int col = [[aTableColumn identifier] intValue];
	if(aTableView == tview){
		PSTATE_CTL_INFO* p = &_pstate_table[rowIndex];
		switch(col){
			case 0:
				return [NSString stringWithFormat:@"%d",rowIndex];
			case 1:
				return [NSString stringWithFormat:@"%d",(int)p->frequency];
			case 2:
				return [NSString stringWithFormat:@"%d",(int)p->voltage];
			case 3:
				return [NSString stringWithFormat:@"%02X",p->fid];
			case 4:
				return [NSString stringWithFormat:@"%02X",p->vid];
			case 5:
				return [NSString stringWithFormat:@"%d",p->did];
		}
		return [NSString stringWithFormat:@"Invalid Col %d",col];
	}
	if(aTableView == sview){
		if(col == 1){
			switch(rowIndex){
				case 0:
					return @"Load (%)";
				case 1:
					return @"PState";
				case 2:
					return @"Frequency (MHz)";
				case 3:
					return @"Voltage (mV)";
				case 4:
					return @"Temperature (C)";
			}
		} else {
			switch(rowIndex){
				case 0:
					return [NSString stringWithFormat:@"%.2f",_cpuload * 100];
				case 1:
					return [NSString stringWithFormat: @"%d / %d",_info.pstate, _pstateCount-1];
				case 2:
					return [NSString stringWithFormat: @"%d (%d%%)",_info.frequency, (_info.frequency * 100 + _pstate_table[0].frequency/2) / _pstate_table[0].frequency];
				case 3:
					return [NSString stringWithFormat: @"%d",_info.voltage];
				case 4:
					return [NSString stringWithFormat: @"%d",_info.temperature];
			}
		}
	}
	return nil;
}

-(void)tableViewSelectionDidChange:(NSNotification *)aNotification {
	autoThrottle = NO;
	fixedState = [tview selectedRow];
	[autoCheckBox setState:NSOffState];
}

-(IBAction)checked:(id)sender {
	if(sender == autoCheckBox){
		autoThrottle = [autoCheckBox state] == NSOnState;
		if(autoThrottle){
			// [tview deselectRow:[tview selectedRow]];
		} else {
			[tview selectRowIndexes:[NSIndexSet indexSetWithIndex: 0] byExtendingSelection:NO];
			fixedState = 0;
		}
	}
}

-(IBAction)pushed:(id)sender {
	PSTATECHANGER_PREF pref;
	if(sender == saveButton || sender == applyButton){
		[self extractForm:&pref];
		if( checkPref(&pref, _pstateCount, getPstateInterval()) == 0){
			_pref = pref;
			// reset interval
			if(updateTimer){
				[updateTimer invalidate];
				[updateTimer release];
			}
			updateTimer = [[NSTimer scheduledTimerWithTimeInterval:(float)_pref.interval / 1000
					target:self selector:@selector(updateInfo:)
					userInfo:nil repeats:YES] retain];
			if(sender == applyButton)
				savePref(&pref);
		} else {
			NSBeep();
		}
	}
	if(sender == loadButton){
		loadPref(&pref, _pstateCount);
		[self fillForm:&pref];
	}
	if(sender == defaultButton){
		defaultPref(&pref, _pstateCount);
		[self fillForm:&pref];
	}
}

@end
