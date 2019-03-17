const person = {
    name: '',
    mass: 0,
    height: 0,
    BMI: function() {
        return this.mass / (this.height**2);
    },
    toString: function() {
        console.log(`My name is ${this.name} and my BMI is ${this.BMI()}`)
    }
};

const mark = Object.create(person);
mark.name = 'mark';
mark.mass = 100;
mark.height = 50;
mark.toString();


const john = Object.create(person);
john.name = 'john';
john.mass = 2000;
john.height = 200;
john.toString();

let greaterBMI = mark.BMI() > john.BMI();
console.log(`is Mark's bmi of ${mark.BMI()} greater than john's BMI of ${john.BMI()}? ${greaterBMI}`);
