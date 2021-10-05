#include "reconos_calls.h"
#include "reconos_thread.h"

#include <stdint.h>


#include "hls_video.h"


using namespace hls;

const int EDGE_THRESHOLD = 19;

const int iniThFAST = 20;
//const int minThFAST = 7;
const int minThFAST = 14;

#define W 30

#define FAST_WINDOW_SIZE 50

#define IMAGE_CACHE_WIDTH   1280
#define IMAGE_CACHE_HEIGHT  150



//generate array 
template<int PSize,int KERNEL_SIZE, int N, int SRC_T,int ROWS,int COLS>
void _FAST_t_opr(
        hls::Mat<ROWS,COLS,SRC_T>    &_src,
        uint32_t (&_vKeysCell)[N],
        HLS_TNAME(SRC_T)        _threshold,
        bool                    _nonmax_supression,
        int                     (&flag)[PSize][2], 
        uint32_t &nPoints,   
        uint32_t x_offset,
        uint32_t y_offset
        )
{
    typedef typename pixel_op_type<HLS_TNAME(SRC_T)>::T INPUT_T;
    LineBuffer<KERNEL_SIZE-1,COLS,INPUT_T>    k_buf;
    LineBuffer<2,COLS+KERNEL_SIZE,ap_int<16> >         core_buf;
    Window<3,3,ap_int<16> >                            core_win;
    Window<KERNEL_SIZE,KERNEL_SIZE,INPUT_T>       win;
    Scalar<HLS_MAT_CN(SRC_T), HLS_TNAME(SRC_T)>             s;
    int rows= _src.rows;
    int cols= _src.cols;
    assert(rows <= ROWS);
    assert(cols <= COLS);
    int kernel_half=KERNEL_SIZE/2;
    ap_uint<2> flag_val[PSize+PSize/2+1];
    int  flag_d[PSize+PSize/2+1];
#pragma HLS ARRAY_PARTITION variable=flag_val dim=0
#pragma HLS ARRAY_PARTITION variable=flag_d dim=0
    uint32_t index=0;
    int offset=KERNEL_SIZE/2;

    if(_nonmax_supression)
    {
        offset=offset+1;
    }
 loop_height: for(HLS_SIZE_T i=0;i<rows+offset;i++) {
#pragma HLS UNROLL factor=10
    loop_width: for(HLS_SIZE_T j=0;j<cols+offset;j++) {
#pragma HLS LOOP_FLATTEN off
#pragma HLS PIPELINE II=1
            if(i<rows&&j<cols) {
                for(int r= 0;r<KERNEL_SIZE;r++) {
                    for(int c=0;c<KERNEL_SIZE-1;c++) {
                        win.val[r][c]=win.val[r][c+1];//column left shift
                    }
                }
                win.val[0][KERNEL_SIZE-1]=k_buf.val[0][j];
                for(int buf_row= 1;buf_row< KERNEL_SIZE-1;buf_row++) {
                    win.val[buf_row][KERNEL_SIZE-1]=k_buf.val[buf_row][j];
                    k_buf.val[buf_row-1][j]=k_buf.val[buf_row][j];
                }
                //-------
                _src>>s;
                win.val[KERNEL_SIZE-1][KERNEL_SIZE-1]=s.val[0];
                k_buf.val[KERNEL_SIZE-2][j]=s.val[0];
            }
            //------core
            for(int r= 0;r<3;r++)
            {
                for(int c=0;c<3-1;c++)
                {
                    core_win.val[r][c]=core_win.val[r][c+1];//column left shift
                }
            }
            core_win.val[0][3-1]=core_buf.val[0][j];
            for(int buf_row= 1;buf_row< 3-1;buf_row++)
            {
                core_win.val[buf_row][3-1]=core_buf.val[buf_row][j];
                core_buf.val[buf_row-1][j]=core_buf.val[buf_row][j];
            }
            int core=0;
            //output
            //if(i>=KERNEL_SIZE-1&&j>=KERNEL_SIZE-1)
            if(i>=KERNEL_SIZE-1 && i<rows && j>=KERNEL_SIZE-1 & j<cols)
            {
                //process
                bool iscorner=fast_judge<PSize>(win,(INPUT_T)_threshold,flag_val,flag_d,flag,core,_nonmax_supression);
                if(iscorner&&!_nonmax_supression)
                {
                        if(index<N)
                        {
                            //_keypoints[index].x=j-offset;
                            //_keypoints[index].y=i-offset;

                            _vKeysCell[index] = (((uint32_t)(j-offset)) + x_offset) | ((((uint32_t)(i-offset)) + y_offset) << 16);
                            index++;
                        }
                }
            }
            if(i>=rows||j>=cols)
            {
                core=0;
            }
            if(_nonmax_supression)
            {
                core_win.val[3-1][3-1]=core;
                core_buf.val[3-2][j]=core;
                if(i>=KERNEL_SIZE&&j>=KERNEL_SIZE&&core_win.val[1][1]!=0)
                {
                    bool iscorner=fast_nonmax(core_win);
                    if(iscorner)
                    {
                        if(index<N)
                        {
                            //_keypoints[index].x=j-offset;
                            //_keypoints[index].y=i-offset;
                            _vKeysCell[index] = (((uint32_t)(j-offset)) + x_offset) | ((((uint32_t)(i-offset)) + y_offset) << 16);
                            index++;
                        }
                    }
                }
            }

        }
    }

	nPoints = index;
}

template<int N, int SRC_T,int ROWS,int COLS>
void  _FASTX(
        Mat<ROWS,COLS,SRC_T>    &_src,
        uint32_t (&_vKeysCell)[N],
        HLS_TNAME(SRC_T)    _threshold,
        bool   _nomax_supression,
        uint32_t &_nPoints,
        uint32_t x_offset,
        uint32_t y_offset
        )
{
#pragma HLS INLINE
    int flag[16][2]={{3,0},{4,0},{5,1},{6,2},{6,3},{6,4},{5,5},{4,6},
        {3,6},{2,6},{1,5},{0,4},{0,3},{0,2},{1,1},{2,0}};
    _FAST_t_opr<16,7>(_src,_vKeysCell,_threshold,_nomax_supression,flag,_nPoints, x_offset, y_offset);
}




void mat_copy( uint8_t data[][IMAGE_CACHE_WIDTH], hls::Mat<FAST_WINDOW_SIZE,FAST_WINDOW_SIZE,HLS_8UC1>  &_dest, int iniY, int iniX, int cache_cnt,uint32_t image_ptr_offset, uint32_t image_width)
{
    for(int i = 0; i < (FAST_WINDOW_SIZE); i++)
    {
        uint32_t line_index = (iniY+i);
        uint32_t offset = (image_ptr_offset +  line_index*(image_width))&3;

        for(int j = 0; j < (FAST_WINDOW_SIZE); j++)
        {
            _dest.write(data[line_index%IMAGE_CACHE_HEIGHT][offset + (j+iniX)]);
        }
    }
}



#define read_next_lines {for(int __i = 0; __i < FAST_WINDOW_SIZE; __i++){ \
                                    uint32_t _offset = (image_ptr + cache_cnt*(image_width))&3; \
                                    MEM_READ(((image_ptr + cache_cnt*(image_width))&(~3)), image_data[(cache_cnt%IMAGE_CACHE_HEIGHT)],((image_width+_offset+3)&(~3)));\
                                    cache_cnt+=1;} }



THREAD_ENTRY() {

    uint32_t nresults;

    THREAD_INIT();
	uint32_t initdata = GET_INIT_DATA();


    uint8_t image_data[IMAGE_CACHE_HEIGHT][IMAGE_CACHE_WIDTH];
    #pragma HLS array_partition variable=image_data block  factor=50  dim=1
	while(1)
	{
        uint32_t cache_cnt = 0;
        uint32_t nWrittenPoints = 0;
        uint32_t image_ptr    = MBOX_GET(initdata*2);
        uint32_t image_width  = MBOX_GET(initdata*2);
        uint32_t image_height = MBOX_GET(initdata*2);
        uint32_t feature_dest = MBOX_GET(initdata*2);

        uint32_t image_ptr_offset = image_ptr & 3;
	    const int minBorderX = EDGE_THRESHOLD-3;
        const int minBorderY = minBorderX;
        const int maxBorderX = image_width -EDGE_THRESHOLD+3;
        const int maxBorderY = image_height-EDGE_THRESHOLD+3;

        const int width = (maxBorderX-minBorderX);
        const int height = (maxBorderY-minBorderY);

        const int wCell = 50;
        const int hCell = 50;
        const int nCols = width/50;
        const int nRows = height/50;

        read_next_lines;

	for(int i=0; i<nRows; i++)
	{
		read_next_lines;

		const int iniY =minBorderY+i*hCell; //this was float
		int maxY = iniY+hCell+6; //this was float

		if(iniY>=maxBorderY-3)
			continue;
		if(maxY>maxBorderY)
			maxY = maxBorderY;

		for(int j=0; j<nCols; j++)
		{
	               const int iniX =minBorderX+j*wCell; //this was float
			int maxX = iniX+wCell+6;//this was float
			if(iniX>=maxBorderX-6)
				continue;
			if(maxX>maxBorderX)
				maxX = maxBorderX;

			uint32_t vKeysCell[256];
        		uint32_t nPoints;

        		hls::Mat<FAST_WINDOW_SIZE,FAST_WINDOW_SIZE,HLS_8UC1> cell_stream(FAST_WINDOW_SIZE,FAST_WINDOW_SIZE);
                #pragma HLS stream depth=2500 variable=cell_stream.data_stream

	                {
            			nPoints = 0;
            			mat_copy( image_data, cell_stream, iniY,  iniX,  cache_cnt, image_ptr_offset, image_width);
            			#pragma HLS INLINE
            			_FASTX(cell_stream, vKeysCell,  minThFAST, true, nPoints, j*wCell, i*hCell); 
            			MEM_WRITE(vKeysCell,feature_dest,nPoints*4);
            			feature_dest+=(4*nPoints);
            			nWrittenPoints+= nPoints;
	               }

        		    /*
				if(nPoints == 0)
			{

                    //#pragma HLS stream depth=200 variable=cell_stream.data_stream
                    {
                        #pragma HLS dataflow
                        #pragma HLS INLINE
                        mat_copy( image_data, cell_stream, iniY,  iniX,  image_width);
                        //FPGA::FPGA_FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),	vKeysCell,minThFAST,true);
                        nPoints = _FASTX<uint16, 256,HLS_8UC1, FAST_WINDOW_SIZE,FAST_WINDOW_SIZE>( cell_stream, vKeysCell,  iniThFAST, true);
                    }
				}

                */		}
	}

        MBOX_PUT(initdata*2+1, nWrittenPoints);
	}

}
