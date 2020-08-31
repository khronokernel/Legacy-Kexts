//
//  LoadChart.h
//  PStateChanger
//
//

#import <Cocoa/Cocoa.h>


@interface LoadChart : NSView {
	int _count, _start;
	float* _data;
	float* _pstate;
	float _frameWidth;
}

- (void)setData:(float*)data pstate:(float*)pstate count:(int)count start:(int)start;
- (void)setFrameWidth:(float)aValue;
@end
