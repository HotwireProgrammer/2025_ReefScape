package frc.robot;

import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class Window {
    public int mouse_y;
    public int mouse_x;
    public boolean leftDown = false;
    public boolean rightDown = false;
    public boolean centerDown = false;
    public final int width, height;
    private final JFrame frame;
    private final Canvas canvas;
    private final BufferedImage grid;
    public final Graphics2D graphics;
    public Color pxColorOut;


    public Window(String title, Color defaultColor, int width, int height) {
        this.width = width;
        this.height = height;

        frame = new JFrame(title);
        frame.setSize(width, height);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setResizable(false);

        canvas = new Canvas();
        canvas.setSize(width, height);
        frame.add(canvas);
        frame.setVisible(true);

        canvas.createBufferStrategy(2);
        grid = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        graphics = grid.createGraphics();

    }

    private BufferedImage loadImage(String path) {
        try {
            return ImageIO.read(new File(path));
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    private Map<String, BufferedImage> imageCache = new HashMap<>();

    public void addImage(String path, double scaleFactor, int x, int y) {
        BufferedImage img = imageCache.get(path);
        if (img == null) {
            img = loadImage(path);
            if (img != null) {
                imageCache.put(path, img);
            }
        }

        if (img != null) {
            if (scaleFactor != 1.0) {
                int newWidth = (int) (img.getWidth() * scaleFactor);
                int newHeight = (int) (img.getHeight() * scaleFactor);
                graphics.drawImage(img, x, y, newWidth, newHeight, null);
            } else {
                graphics.drawImage(img, x, y, null);
            }
        }
    }


    public void setIcon(String path) {
        try {
            Image icon = ImageIO.read(new File(path));
            frame.setIconImage(icon);
            Taskbar.getTaskbar().setIconImage(icon);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void show() {
        frame.setVisible(true);
    }

    public void hide() {
        frame.setVisible(false);
    }
}
