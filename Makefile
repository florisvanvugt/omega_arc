#default: arc
default: rob test

quit:
	make -C robot kill

kill: # Kill the robot, reasonably gently
	make -C robot kill

doc: documentation.html
	xdg-open documentation.html

documentation.html: readme.md fonts/github-pandoc.css
	pandoc -f markdown -t html readme.md -s -c fonts/github-pandoc.css -o documentation.html

straight:
	python run_straight.py

simpleschedule.txt: make_schedule.py
	python make_schedule.py

test: robot simpleschedule.txt
	python run_arc.py test simpleschedule.txt

arc: rob
	python run_arc.py

training: robot
	python run_arc.py training training.schedule.par


rob:
	make -C robot


clean:
	rm -f *.pyc
	rm -f *~
	rm -f documentation.html
	make -C robot clean
