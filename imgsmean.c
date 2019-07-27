/**
 * Tool that computes the average of a list of images
 *
 * @version $Revision$$Date::             $
 * @copyright Copyright (c) 2015 to the present, Mauricio Villegas <mauvilsa@upv.es>
 */

/*** Includes *****************************************************************/
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <wand/magick_wand.h>

#include "log.h"

/*** Definitions **************************************************************/
static char tool[] = "imgsmean";
static char revnum[] = "$Revision$";
static char revdate[] = "$Date$";

char *ofn = NULL;
FILE *infofile = NULL;
FILE *logfile = NULL;
int verbosity = 1;

int persp = FALSE;
double gb_fact_ul = 0.08;
double gb_fact_dr = 0.08;

int imgW = 0;
int imgH = 0;
int resize = FALSE;

/*** Functions ****************************************************************/
void print_usage( FILE *file ) {
  fprintf( file, "Usage: %s [options] <out_img>\n", tool );
  fprintf( file, "\n" );
  fprintf( file, " -i LSTFILE      File with input list of images (def.=stdin)\n" );
  fprintf( file, " -r WxH          Resize images to width W and height H (def.=false)\n" );
  fprintf( file, " -P oX,oY        Perspective distortion with oX,oY sides offset (def.=false)\n" );
  fprintf( file, " -l lfile        Logging to 'lfile' (def.=stderr)\n" );
  fprintf( file, " -V (-|+|level)  Set verbosity level (def.=%d)\n", verbosity );
  fprintf( file, " -h              Print this usage information and exit\n" );
  fprintf( file, " -v              Print version and exit\n" );
  fprintf( file, "\n");
  print_svn_rev( file );
}

void extend_line( double p1x, double p1y, double p2x, double p2y, double fact, double* _px, double* _py ) {
  double l = sqrt( (p2x-p1x)*(p2x-p1x) + (p2y-p1y)*(p2y-p1y) );
  double mx = (p2x-p1x)/l;
  double my = (p2y-p1y)/l;
  *_px = ( fact < 0 ? p1x : p2x ) + fact*l*mx;
  *_py = ( fact < 0 ? p1y : p2y ) + fact*l*my;
}


/*** Program ******************************************************************/
int main( int argc, char *argv[] ) {
  logfile = stderr;
  infofile = stdin;
  int err = 0;

  /// Parse input arguments ///
  if( ! strncmp( "--", argc>1 ? argv[1] : "", 2 ) ) {
    print_svn_rev( logfile );
    return SUCCESS;
  }

  char *optstr = "i:r:P:l:V:hv";
  while( getopt(argc,argv,optstr) != -1 );
  int nopts = optind;
  optind = 1;

  int n;
  while( ( n = getopt(argc,argv,optstr) ) != -1 )
    switch( n ) {
    case 'i':
      if( (infofile = fopen(optarg,"r")) == NULL )
        die( "error: unable to %s file %s", "open info", optarg );
      break;
    case 'r':
      resize = TRUE;
      imgW = atoi(optarg);
      imgH = strchr(optarg,'x') == NULL ? 0 : atoi(strchr(optarg,'x')+1);
      if( imgW <= 0 || imgH <= 0 )
        die( "error: unexpected resize dimensions %dx%d", imgW, imgH );
      break;
    case 'P':
      persp = TRUE;
      gb_fact_ul = atof(optarg);
      gb_fact_dr = strchr(optarg,',') != NULL ?
        atof(strchr(optarg,',')+1) :
        gb_fact_ul ;
      logger( 2, "perspective distortion mode %g,%g", gb_fact_ul, gb_fact_dr );
      break;
    case 'l':
      if( ! strcmp( optarg, "-" ) ||
          ! strcmp( optarg, "stdout" ) ||
          ! strcmp( optarg, "/dev/stdout" ) )
        logfile = stdout;
      else if( (logfile = fopen(optarg,"ab")) == NULL )
        die( "error: unable to %s file %s", "open log", optarg );
      break;
    case 'V':
      if( optarg[0] == '+' )
        verbosity ++;
      else if( optarg[0] == '-' )
        verbosity --;
      else
        verbosity = atoi( optarg );
      verbosity = verbosity < 0 ? 0 : verbosity;
      break;
    default:
      logger( 0, "error: incorrect input argument (-%c)", n );
      err = FAILURE;
    case 'h':
      print_usage( logfile );
      return err;
    case 'v':
      print_svn_rev( logfile );
      return err;
    }

  if( argc - nopts > 2 ) {
    logger( 0, "error: expected at most two non-option arguments" );
    print_usage( logfile );
    return FAILURE;
  }

  if( argc - nopts > 0 )
    ofn = argv[optind];

  if( ofn == NULL ) {
    logger( 0, "error: output image required" );
    print_usage( logfile );
    return FAILURE;
  }

  MagickWandGenesis();
  MagickWand *img = NULL;
  double *average = NULL;
  int channels = 0;

  /// Loop through image list accumulating ///
  int m = 0;
  char *line = NULL;
  size_t linesz = 0;
  while( getline( &line, &linesz, infofile ) > 1 ) {
    if( strchr(line,'\n') != NULL )
      *( strchr(line,'\n') ) = '\0';

    m++;

    if( img != NULL )
      img = DestroyMagickWand(img);
    img = NewMagickWand();

    char *imgfile = line;
    double ulX, ulY, urX, urY, drX, drY, dlX, dlY;
    if( persp ) {
      int nchars;
      if( 8 != sscanf( line, "%lf,%lf %lf,%lf %lf,%lf %lf,%lf %n", 
                 &ulX, &ulY,
                 &urX, &urY,
                 &drX, &drY,
                 &dlX, &dlY,
                 &nchars ) )
        die( "error: expected 4 x,y coordinates preceding the image file name %d: %s", m, line );
      imgfile = line+nchars;
    }

    /// Read image ///
    if( ! MagickReadImage( img, imgfile ) )
      die( "error: unable to read image %d: %s", m, imgfile );
    logger( 2, "read image %d: %s", m, imgfile );

    /// Perspective distortion ///
    if( persp ) {
      /*| awk '
          function extend_line( p1x, p1y, p2x, p2y, fact, p3,    l,mx,my ) {
            l = sqrt( (p2x-p1x)*(p2x-p1x) + (p2y-p1y)*(p2y-p1y) );
            mx = (p2x-p1x)/l;
            my = (p2y-p1y)/l;
            delete p3;
            p3[1] = ( fact < 0 ? p1x : p2x ) + fact*l*mx;
            p3[2] = ( fact < 0 ? p1y : p2y ) + fact*l*my;
          }

          { ulX = $1; ulY = $2;
            urX = $3; urY = $4;
            drX = $5; drY = $6;
            dlX = $7; dlY = $8;

            fact_ul = 0.08;
            fact_dr = 0.08;

            extend_line( ulX, ulY, urX, urY, -fact_ul, ull );
            extend_line( dlX, dlY, drX, drY, -fact_ul, dll );
            extend_line( ull[1], ull[2], dll[1], dll[2], -fact_ul, uull );
            extend_line( ull[1], ull[2], dll[1], dll[2], fact_dr, ddll );

            extend_line( ulX, ulY, urX, urY, fact_dr, urr );
            extend_line( dlX, dlY, drX, drY, fact_dr, drr );
            extend_line( urr[1], urr[2], drr[1], drr[2], -fact_ul, uurr );
            extend_line( urr[1], urr[2], drr[1], drr[2], fact_dr, ddrr );

            w = uurr[1]-uull[1];
            if( w > ddrr[1]-ddll[1] )
              w = ddrr[1]-ddll[1];
            h = ddrr[2]-uurr[2];
            if( h > ddll[2]-uull[2] )
              h = ddll[2]-uull[2];

            printf("-distort Perspective ");
            printf("%.0f,%.0f_%.0f,%.0f__",uull[1],uull[2],0,0);
            printf("%.0f,%.0f_%.0f,%.0f__",uurr[1],uurr[2],w-1,0);
            printf("%.0f,%.0f_%.0f,%.0f__",ddrr[1],ddrr[2],w-1,h-1);
            printf("%.0f,%.0f_%.0f,%.0f"  ,ddll[1],ddll[2],0,h-1);
            printf(" -crop %.0fx%.0f+0+0",w,h);
          }' ) );*/

      logger( 2, "image %d perspective distortion: %.1g,%.1g %.1g,%.1g %.1g,%.1g %.1g,%.1g", m, ulX, ulY, urX, urY, drX, drY, dlX, dlY );

      double ullX, ullY, dllX, dllY;
      double urrX, urrY, drrX, drrY;
      double uullX, uullY, ddllX, ddllY;
      double uurrX, uurrY, ddrrX, ddrrY;

      extend_line( ulX, ulY, urX, urY, -gb_fact_ul, &ullX, &ullY );
      extend_line( dlX, dlY, drX, drY, -gb_fact_ul, &dllX, &dllY );
      extend_line( ullX, ullY, dllX, dllY, -gb_fact_ul, &uullX, &uullY );
      extend_line( ullX, ullY, dllX, dllY, gb_fact_dr, &ddllX, &ddllY );

      extend_line( ulX, ulY, urX, urY, gb_fact_dr, &urrX, &urrY );
      extend_line( dlX, dlY, drX, drY, gb_fact_dr, &drrX, &drrY );
      extend_line( urrX, urrY, drrX, drrY, -gb_fact_ul, &uurrX, &uurrY );
      extend_line( urrX, urrY, drrX, drrY, gb_fact_dr, &ddrrX, &ddrrY );

      double w = uurrX-uullX;
      if( w > ddrrX-ddllX )
        w = ddrrX-ddllX;
      double h = ddrrY-uurrY;
      if( h > ddllY-uullY )
        h = ddllY-uullY;

      double pargs[16];
      pargs[0] = uullX;  pargs[1] = uullY;  pargs[2] = 0.0;  pargs[3] = 0.0;
      pargs[4] = uurrX;  pargs[5] = uurrY;  pargs[6] = w-1;  pargs[7] = 0.0;
      pargs[8] = ddrrX;  pargs[9] = ddrrY;  pargs[10] = w-1; pargs[11] = h-1;
      pargs[12] = ddllX; pargs[13] = ddllY; pargs[14] = 0.0; pargs[15] = h-1;

      int xmin = -1, xmax = -1, ymin = -1, ymax = -1;
      int k;
      for( k=0; k<16; k+=4 ) {
        int pxfloor = (int)floor(pargs[k]);
        int pxceil = (int)ceil(pargs[k]);
        int pyfloor = (int)floor(pargs[k+1]);
        int pyceil = (int)ceil(pargs[k+1]);
        xmin = k==0 || pxfloor < xmin ? pxfloor : xmin ;
        xmax = k==0 || pxceil > xmax ? pxceil : xmax ;
        ymin = k==0 || pyfloor < ymin ? pyfloor : ymin ;
        ymax = k==0 || pyceil > ymax ? pyceil : ymax ;
      }
      int imW = (int)MagickGetImageWidth(img);
      int imH = (int)MagickGetImageHeight(img);

      if( xmin > 0 || ymin > 0 || xmax < imW-1 || ymax < imH-1 ) {
        xmin = xmin < 0 ? 0 : xmin ;
        ymin = ymin < 0 ? 0 : ymin ;
        xmax = xmax >= imW ? imW-1 : xmax ;
        ymax = ymax >= imH ? imH-1 : ymax ;
        for( k=0; k<16; k+=4 ) {
          pargs[k] -= xmin;
          pargs[k+1] -= ymin;
        }
        MagickCropImage( img, xmax-xmin+1, ymax-ymin+1, xmin, ymin );
        logger( 3, "image %d pre-distortion crop: %dx%d+%d+%d", m, xmax-xmin+1, ymax-ymin+1, xmin, ymin );
      }

      MagickDistortImage( img, PerspectiveDistortion, 16, pargs, MagickFalse );
      MagickCropImage( img, (int)(w+0.5), (int)(h+0.5), 0, 0 );
      logger( 3, "image %d post-distortion crop: %dx%d+0+0", m, (int)(w+0.5), (int)(h+0.5) );
    }

    /// Check image dimensions and resize if required ///
    if( imgW == 0 ) {
      imgW = MagickGetImageWidth(img);
      imgH = MagickGetImageHeight(img);
    }
    if( imgW != (int)MagickGetImageWidth(img) ||
        imgH != (int)MagickGetImageHeight(img) ) {
      if( ! resize )
        die( "error: expected size of image %d to be %dx%d and is %dx%d: %s", m, imgW, imgH, (int)MagickGetImageWidth(img), (int)MagickGetImageHeight(img), imgfile );
      logger( 2, "image %d resize: %dx%d => %dx%d", m, (int)MagickGetImageWidth(img), (int)MagickGetImageHeight(img), imgW, imgH );
      MagickResizeImage( img, imgW, imgH, LanczosFilter, 1 );
    }

    if( average == NULL ) {
//Bilevel        Grayscale       GrayscaleMatte
//Palette        PaletteMatte    TrueColor
//TrueColorMatte ColorSeparation ColorSeparationMatte
      /*if( MagickGetImageType(img) == Bilevel ||
          MagickGetImageType(img) == Grayscale ||
          MagickGetImageType(img) == GrayscaleMatte )
        channels = 1;
      else if( MagickGetImageType(img) == TrueColor
               MagickGetImageType(img) == TrueColorMatte )*/
        channels = 3;
      if( (average=(double*)calloc(imgW*imgH*channels,sizeof(double))) == NULL )
        die( "error: unable to reserve memory" );
    }

    /// Add values of current image ///
    PixelIterator *iterator = NewPixelIterator(img);
    n = 0;
    int y;
    for( y=0; y<imgH; y++ ) {
      size_t number_wands;
      PixelWand **pixels = PixelGetNextIteratorRow( iterator, &number_wands );
      int x;
      for( x=0; x<imgW; x++ ) {
        //if( channels == 1 )
        //  average[n++] += PixelGetBlack(pixels[x]);
        //else {
          average[n++] += PixelGetRed(pixels[x]);
          average[n++] += PixelGetGreen(pixels[x]);
          average[n++] += PixelGetBlue(pixels[x]);
        //}
      }
    }
    iterator = DestroyPixelIterator(iterator);
  }

  /// Set average image to wand ///
  PixelIterator *iterator = NewPixelIterator(img);
  n = 0;
  int y;
  for( y=0; y<imgH; y++ ) {
    size_t number_wands;
    PixelWand **pixels = PixelGetNextIteratorRow( iterator, &number_wands );
    int x;
    for( x=0; x<imgW; x++ ) {
      //if( channels == 1 )
      //  PixelSetBlack( pixels[x], average[n++]/m );
      //else {
        PixelSetRed( pixels[x], average[n++]/m );
        PixelSetGreen( pixels[x], average[n++]/m );
        PixelSetBlue( pixels[x], average[n++]/m );
      //}
    }
    (void) PixelSyncIterator(iterator);
  }
  (void) PixelSyncIterator(iterator);
  iterator = DestroyPixelIterator(iterator);

  /// Write output image ///
  logger( 1, "averaged %d input images", m );
  logger( 1, "output image: %s", ofn );
  MagickWriteImage( img, ofn );

  /// Release resources ///
  if( infofile != stdin )
    fclose( infofile );
  free(line);
  free(average);
  img = DestroyMagickWand( img );
  MagickWandTerminus();

  return SUCCESS;
}
