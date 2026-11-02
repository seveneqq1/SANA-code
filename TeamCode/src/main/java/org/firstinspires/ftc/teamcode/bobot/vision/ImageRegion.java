 
































package org.firstinspires.ftc.teamcode.bobot.vision;

import com.qualcomm.robotcore.util.Range;
import org.opencv.core.Rect;

 







public class ImageRegion
{
    final boolean imageCoords;
    final double left, top, right, bottom;

     







    private ImageRegion(boolean imageCoords, double left, double top, double right, double bottom )
    {
        this.left = left;
        this.top = top;
        this.right = right;
        this.bottom = bottom;
        this.imageCoords = imageCoords;

        if (imageCoords)
        {
            if (left >= right || top >= bottom || left < 0 || top < 0)
            {
                throw new IllegalArgumentException("Malformed ImageRegion boundaries!");
            }
        }
        else  
        {
            if (left >= right || top <= bottom || left < -1.0 || right > 1.0 || top > 1.0 || bottom < -1.0)
            {
                throw new IllegalArgumentException("Malformed ImageRegion boundaries!");
            }
        }
    }

     

















    public static ImageRegion asImageCoordinates(int left, int top, int right, int bottom )
    {
        return new ImageRegion(true, left, top, right, bottom);
    }

     

















    public static ImageRegion asUnityCenterCoordinates(double left, double top, double right, double bottom)
    {
        return new ImageRegion(false, left, top, right, bottom);
    }

     



    public static ImageRegion entireFrame()
    {
        return ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1);
    }

     







    protected Rect asOpenCvRect(int imageWidth, int imageHeight)
    {
        Rect rect = new Rect();

        if (imageCoords)
        {
            rect.x = (int) left;
            rect.y = (int) top;
            rect.width = (int) (right - left);
            rect.height = (int) (bottom - top);
        }
        else  
        {
            rect.x = (int) Range.scale(left, -1, 1, 0, imageWidth);
            rect.y = (int) ( imageHeight - Range.scale(top, -1, 1, 0, imageHeight));
            rect.width = (int) Range.scale(right - left, 0, 2, 0, imageWidth);
            rect.height = (int) Range.scale(top - bottom, 0, 2, 0, imageHeight);
        }

         
         
        rect.x = Math.max(rect.x, 0);
        rect.x = Math.min(rect.x, imageWidth - rect.width);
        rect.y = Math.max(rect.y, 0);
        rect.y = Math.min(rect.y, imageHeight - rect.height);

        return rect;
    }
}
