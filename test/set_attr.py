class Baskets:
  fruit = 'Banana'
  vegetable = 'Potato'

bas = Baskets()

# Adding candy to the basket
setattr(bas, 'candy', 'Kit Kat')
print('Added:', bas.candy)