//
//  PStateChangerAppDelegate.h
//  PStateChanger
//
//

#import <Cocoa/Cocoa.h>

@class LoadChart;
#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_5
@interface PStateChangerAppDelegate : NSObject {
#else
	@interface PStateChangerAppDelegate : NSObject <NSApplicationDelegate,NSTableViewDataSource,NSTableViewDelegate> {
#endif
    NSWindow *window;
    IBOutlet LoadChart *lcview;
    IBOutlet NSTableView *tview;
    IBOutlet NSTableView *sview;
    IBOutlet NSButton *autoCheckBox;
	IBOutlet NSForm* prefForm;
	IBOutlet NSButton* applyButton;
	IBOutlet NSButton* saveButton;
	IBOutlet NSButton* loadButton;
	IBOutlet NSButton* defaultButton;
	NSTimer *updateTimer;

    LoadChart *docview;
	Boolean autoThrottle;
	int fixedState;
}

@property (assign) IBOutlet NSWindow *window;
-(IBAction)checked:(id)sender;
-(IBAction)pushed:(id)sender;

@end
