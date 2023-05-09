Sorry for so many files, I've been doing a lot of things.


The things you'll want to look at are in rufous_hinf.m, rufous_h2.m and mitchells_hand_tuned.m. 

Those should work without any other files. The live scripts do different things rufous.mlx is the closest to a project write write up. The live scripts all depend on the functions.

I have designed three controllers, Hinf, H2 and a hand tuned one. The Hinf was the only one that achieved stability but the hand tuned was the only one that achieved performance. H2 was a bit disappointing, the controller does not work due to z have feedforward properties from the reference qr (z = qr-q).

I'm interested to see a genereated controller (H2, Hinf, or whatever) that performs better. I feel like I need to sacrifice some stability to achieve this and maybe the Hinf LMI doesn't want to do that.
