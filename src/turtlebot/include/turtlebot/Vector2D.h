/*
Date: 19.12.2023
Developed by: Ahmet Furkan Akıncı
Project: EE 451 - Project 4
Summary: Includes a library to do linear 2x1 vector operations
*/

#ifndef VECTOR2D_H
#define VECTOR2D_H

#include <math.h>

class Vector2D {
public:
    double x, y;

    /*
    Date: 19.12.2023
    Developed by: Ahmet Furkan Akıncı
    Summary: Default constructor for Vector2D, initializes x and y to zero.
    Input: N/A
    Output: N/A
    Additional info: Useful for creating a zero vector
    */
    Vector2D() {
        x = 0;
        y = 0;
    }

    /*
    Date: 19.12.2023
    Developed by: Ahmet Furkan Akıncı
    Summary: Constructor for Vector2D that initializes x and y with given values.
    Input: double _x, double _y - The values to initialize x and y
    Output: N/A
    Additional info: Enables creating a vector with custom coordinates
    */
    Vector2D(double _x, double _y) {
        x = _x; 
        y = _y; 
    }

    /*
    Date: 19.12.2023
    Developed by: Ahmet Furkan Akıncı
    Summary: Adds two Vector2D objects.
    Input: const Vector2D& rhs - The vector to add
    Output: Vector2D - The result of the addition
    Additional info: The addition is component-wise
    */
    Vector2D add(const Vector2D& rhs) {
        return Vector2D(x + rhs.x, y + rhs.y);
    }

    /*
    Date: 19.12.2023
    Developed by: Ahmet Furkan Akıncı
    Summary: Subtracts one Vector2D from another.
    Input: const Vector2D& rhs - The vector to subtract
    Output: Vector2D - The result of the subtraction
    Additional info: The subtraction is component-wise
    */
    Vector2D substract(const Vector2D& rhs) {
        return Vector2D(x - rhs.x, y - rhs.y);
    }

    /*
    Date: 19.12.2023
    Developed by: Ahmet Furkan Akıncı
    Summary: Multiplies the Vector2D with a scalar.
    Input: double rhs - The scalar to multiply with
    Output: Vector2D - The result of the multiplication
    Additional info: Both x and y components are multiplied by the scalar
    */
    Vector2D multiply(double rhs) {
        return Vector2D(x * rhs, y * rhs);
    }

    /*
    Date: 19.12.2023
    Developed by: Ahmet Furkan Akıncı
    Summary: Calculates the length (magnitude) of the vector.
    Input: N/A
    Output: double - The length of the vector
    Additional info: Uses the Euclidean norm formula
    */
    double getLength() {
        return sqrt(x * x + y * y);
    }

    /*
    Date: 19.12.2023
    Developed by: Ahmet Furkan Akıncı
    Summary: Gets the unit vector in the direction of the Vector2D.
    Input: N/A
    Output: Vector2D - The unit vector
    Additional info: The unit vector has the same direction but a length of 1
    */
    Vector2D getUnitVector() {
        return Vector2D(x, y).multiply(1 / getLength());
    }
};

#endif //VECTOR2D_HH

