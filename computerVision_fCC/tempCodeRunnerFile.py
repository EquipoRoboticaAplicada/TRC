blur = cv.GaussianBlur(img, (15,1), cv.BORDER_DEFAULT) # matriz en impares a fuerzas
blur2 = cv.GaussianBlur(img, (1,15), cv.BORDER_DEFAULT) # matriz en impares a fuerzas
cv.imshow('blur_x', blur)
cv.imshow('blur_y', blur2)