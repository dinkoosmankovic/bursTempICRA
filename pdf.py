from scipy.stats import norm,beta
from scipy.stats import norm
from numpy import linspace
from pylab import plot,show,hist,figure,title


samp = 5 + beta.rvs(a=2, b=5,size=1500) # samples generation

param = beta.fit(samp) # distribution fitting

x = linspace(5,13,100)

pdf = beta.pdf(x,a=6,b=7)

title('beta distribution')
plot(x,pdf,'b-')
hist(samp,normed=1,alpha=.3)
show()
