#import "ViewController.h"
#import "iQOCWrapper.hpp"

@interface ViewController ()
{
    IQOCWrapper QOCWrapper;
    __weak IBOutlet UIImageView *DepthViewer;
    __weak IBOutlet UITextField *ResultString;
}
@end

@implementation ViewController

- (void)runQOC {
    //self->Console.text = [NSString stringWithCString:QOCWrapper.verify().c_str() encoding:[NSString defaultCStringEncoding]];
    
    self->ResultString.text = [NSString stringWithCString:QOCWrapper.verify().c_str() encoding:[NSString defaultCStringEncoding]];
    
    NSArray  *documentPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDir  = [documentPaths objectAtIndex:0];
    NSString  *pngfile = [documentsDir stringByAppendingPathComponent:@"depth.pgm"];
    
    self->DepthViewer.image = [UIImage imageNamed:pngfile];
}

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
    [self runQOC];
}


@end
