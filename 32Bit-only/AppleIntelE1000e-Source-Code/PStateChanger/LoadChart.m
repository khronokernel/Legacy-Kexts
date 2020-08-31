//
//  LoadChart.m
//  PStateChanger
//
//

#import "LoadChart.h"


@implementation LoadChart
- (void)setFrameWidth:(float)aValue {
	_frameWidth = aValue;
}

- (void)drawRect:(NSRect)dirtyRect {
	[[NSColor blackColor] set];
	NSRect rect = [self bounds];
	NSRectFill(rect);
	//
	if(_data && _count){
		int k;
		CGFloat wid = rect.size.width/_count;
		[[NSColor blueColor] set];
		float level = _pstate[_start % _count];
		int start = 0;
		for( k = 1; k <= _count; k++ ){
			if(k == _count || level != _pstate[(k+_start) % _count]){
				NSRect temp;
				temp.origin.x = rect.origin.x + wid * start - 0.5;
				temp.origin.y = rect.origin.y;
				temp.size.width = wid * (k - start) + 0.5;
				temp.size.height = rect.size.height *level;
				NSRectFill(temp);
				start = k;
				level = _pstate[(k+_start) % _count];
			}
		}
		
		[[NSColor greenColor] set];
		for( k = 0; k < _count; k++ ){
			float load = _data[(k+_start) % _count];
			if( load > 0.0 ){
				NSRect temp;
				temp.origin.x = rect.origin.x + wid * k - 0.5;
				temp.origin.y = rect.origin.y;
				temp.size.width = wid + 0.5;
				temp.size.height = rect.size.height * load;
				NSRectFill(temp);
			}
		}

	}
	if(_frameWidth > 0){
		[[NSColor whiteColor] set];
		NSFrameRectWithWidth(rect, _frameWidth);
	}
}

- (void)setData:(float*)data pstate:(float*)pstate count:(int)count start:(int)start {
	_count = count;
	_start = start;
	_data = data;
	_pstate = pstate;
	[self setNeedsDisplay:YES];
}
@end
