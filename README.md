# Toyota_OBD1_BK_OLED
Бортовой компьютер на arduino.
Давно для просмотра некоторых параметров на наших машинах нужно было подключать ноутбук ну или дорогой мультитроникс . С появлением ардуино в сети нашел несколько вариантов которые можно адаптировать под наш двигатель. За основу был использован проект www.drive2.ru/l/485944283254227602/. В скетче отключена запись логов на sd-карту, пересчитаны формулы под 5A-FE мотор и добавлено еще одно меню с пробегом и с расходом топлива за это расстояние, которое можно обнулить отдельно от других показаний . Точность показания расхода не проверены! БК может показывать: моментальные данные о расходе, средние данные, данные о времени\километраже и экран с данными TOBD1 протокола. Данные можно обнулить долгим нажатием на кнопку.
