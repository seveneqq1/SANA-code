 
































package org.firstinspires.ftc.teamcode.bobot.vision;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

 




public abstract class ColorBlobLocatorProcessor implements VisionProcessor
{
     


    public static class Builder
    {
        private ColorRange colorRange;
        private ContourMode contourMode;
        private ImageRegion imageRegion = ImageRegion.entireFrame();
        private int erodeSize = -1;
        private int dilateSize = -1;
        private boolean drawContours = false;
        private int blurSize = -1;
        private int boundingBoxColor = Color.rgb(255, 120, 31);
        private int roiColor = Color.rgb(255, 255, 255);
        private int contourColor = Color.rgb(3, 227, 252);

         






        public Builder setDrawContours(boolean drawContours)
        {
            this.drawContours = drawContours;
            return this;
        }

         




        public Builder setBoxFitColor(@ColorInt int color)
        {
            this.boundingBoxColor = color;
            return this;
        }

         




        public Builder setRoiColor(@ColorInt int color)
        {
            this.roiColor = color;
            return this;
        }

         




        public Builder setContourColor(@ColorInt int color)
        {
            this.contourColor = color;
            return this;
        }

         




        public Builder setTargetColorRange(ColorRange colorRange)
        {
            this.colorRange = colorRange;
            return this;
        }

         






        public Builder setContourMode(ContourMode contourMode)
        {
            this.contourMode = contourMode;
            return this;
        }

         




        public Builder setRoi(ImageRegion roi)
        {
            this.imageRegion = roi;
            return this;
        }

         






        public Builder setBlurSize(int blurSize)
        {
            this.blurSize = blurSize;
            return this;
        }

         








        public Builder setErodeSize(int erodeSize)
        {
            this.erodeSize = erodeSize;
            return this;
        }

         








        public Builder setDilateSize(int dilateSize)
        {
            this.dilateSize = dilateSize;
            return this;
        }

         





        public ColorBlobLocatorProcessor build()
        {
            if (imageRegion == null)
            {
                throw new IllegalArgumentException("You must set a region of interest!");
            }

            if (colorRange == null)
            {
                throw new IllegalArgumentException("You must set a color range!");
            }

            if (contourMode == null)
            {
                throw new IllegalArgumentException("You must set a contour mode!");
            }

            return new ColorBlobLocatorProcessorImpl(colorRange, imageRegion, contourMode, erodeSize, dilateSize, drawContours, blurSize, boundingBoxColor, roiColor, contourColor);
        }
    }

     


    public enum ContourMode
    {
         


        EXTERNAL_ONLY,

         


        ALL_FLATTENED_HIERARCHY
    }

     


    public enum BlobCriteria
    {
        BY_CONTOUR_AREA,
        BY_DENSITY,
        BY_ASPECT_RATIO,
    }

     


    public static class BlobFilter {
        public final BlobCriteria criteria;
        public final double minValue;
        public final double maxValue;

        public BlobFilter(BlobCriteria criteria, double minValue,  double maxValue)
        {
            this.criteria = criteria;
            this.minValue = minValue;
            this.maxValue = maxValue;
        }
    }

     


    public static class BlobSort
    {
        public final BlobCriteria criteria;
        public final SortOrder sortOrder;

        public BlobSort(BlobCriteria criteria, SortOrder sortOrder)
        {
            this.criteria = criteria;
            this.sortOrder = sortOrder;
        }
    }

     


    public static abstract class Blob
    {
         



        public abstract MatOfPoint getContour();

         



        public abstract Point[] getContourPoints();

         



        public abstract int getContourArea();

         




        public abstract double getDensity();

         




        public abstract double getAspectRatio();

         



        public abstract RotatedRect getBoxFit();
    }

     


    public abstract void addFilter(BlobFilter filter);

     


    public abstract void removeFilter(BlobFilter filter);

     


    public abstract void removeAllFilters();

     


    public abstract void setSort(BlobSort sort);

     



    public abstract List<Blob> getBlobs();

     


    public static class Util
    {
         





        public static void filterByArea(double minArea, double maxArea, List<Blob> blobs)
        {
            ArrayList<Blob> toRemove = new ArrayList<>();

            for(Blob b : blobs)
            {
                if (b.getContourArea() > maxArea || b.getContourArea() < minArea)
                {
                    toRemove.add(b);
                }
            }

            blobs.removeAll(toRemove);
        }

         




        public static void sortByArea(SortOrder sortOrder, List<Blob> blobs)
        {
            blobs.sort(new Comparator<Blob>()
            {
                public int compare(Blob c1, Blob c2)
                {
                    int tmp = (int)Math.signum(c2.getContourArea() - c1.getContourArea());

                    if (sortOrder == SortOrder.ASCENDING)
                    {
                        tmp = -tmp;
                    }

                    return tmp;
                }
            });
        }

         





        public static void filterByDensity(double minDensity, double maxDensity, List<Blob> blobs)
        {
            ArrayList<Blob> toRemove = new ArrayList<>();

            for(Blob b : blobs)
            {
                if (b.getDensity() > maxDensity || b.getDensity() < minDensity)
                {
                    toRemove.add(b);
                }
            }

            blobs.removeAll(toRemove);
        }

         




        public static void sortByDensity(SortOrder sortOrder, List<Blob> blobs)
        {
            blobs.sort(new Comparator<Blob>()
            {
                public int compare(Blob c1, Blob c2)
                {
                    int tmp = (int)Math.signum(c2.getDensity() - c1.getDensity());

                    if (sortOrder == SortOrder.ASCENDING)
                    {
                        tmp = -tmp;
                    }

                    return tmp;
                }
            });
        }

         





        public static void filterByAspectRatio(double minAspectRatio, double maxAspectRatio, List<Blob> blobs)
        {
            ArrayList<Blob> toRemove = new ArrayList<>();

            for(Blob b : blobs)
            {
                if (b.getAspectRatio() > maxAspectRatio || b.getAspectRatio() < minAspectRatio)
                {
                    toRemove.add(b);
                }
            }

            blobs.removeAll(toRemove);
        }

         




        public static void sortByAspectRatio(SortOrder sortOrder, List<Blob> blobs)
        {
            blobs.sort(new Comparator<Blob>()
            {
                public int compare(Blob c1, Blob c2)
                {
                    int tmp = (int)Math.signum(c2.getAspectRatio() - c1.getAspectRatio());

                    if (sortOrder == SortOrder.ASCENDING)
                    {
                        tmp = -tmp;
                    }

                    return tmp;
                }
            });
        }
    }
}
