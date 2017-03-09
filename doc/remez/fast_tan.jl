using Gadfly

println("Minimax test 1 (polynomial):")
(nc, dc, e, x) = ratfn_minimax(x->tan(x), (BigFloat(0), BigFloat(atan(1)/4)), 2, 0)
println("max error ",e*180/pi)
for i=1:length(nc)
  println(string("n[",i,"] = ",nc[i]))
end

f(x) = Remez.poly_eval(nc,BigFloat(x))
println(f(0.1))
println(Remez.poly_eval(nc,BigFloat(0)))

function g(x)
  return f(x)-tan(x)
end

g(0.3)
g(0)

plot([sin], -pi/4, pi/8)
plot([x->f(x)],0,0.2)
#println(f(BigFloat(0))

#println(f(0-.02))
