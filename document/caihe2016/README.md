# 3343 文献中的 mu_1 条件问题

原文 


Let $\tilde { \eta } = \mathsf { c o l } ( \tilde { \eta } _ { 1 } , \dots , \tilde { \eta } _ { N } )$ and $\tilde { S } _ { d } = \mathrm { b l o c k } \mathrm { d i a g } \{ \tilde { S } _ { 1 } , . . . , \tilde { S } _ { N } \}$ . Then (10) can be rewritten into the following compact form

$$
\dot { \tilde { \eta } } = [ ( I _ { N } \otimes S ) - \mu _ { 2 } ( H \otimes I _ { q } ) ] \tilde { \eta } + \tilde { S } _ { d } \tilde { \eta } + \tilde { S } _ { d } ( 1 _ { N } \otimes v ) .\tag{11}
$$

If none of the eigenvalues of S has positive real parts, then, for any positive $\mu _ { 1 }$ and $\mu _ { 2 } ,$ the matrix $( I _ { N } \otimes S ) \ : - \ : \mu _ { 2 } ( H \otimes I _ { q } )$ is Hurwitz (Su & Huang, 2012a) and $\widetilde { S } _ { d } ( t ) ( 1 _ { N } \otimes v ( t ) )$ will decay to zero exponentially. 
If some of the eigenvalues of S have positive real parts, let $\gamma ~ = ~ \operatorname* { m a x } ( \Re ( \sigma ( S ) ) )$ . Then, for $\mu _ { 2 } > \frac { \gamma } { \alpha _ { H } }$ and $\color{red} \mu _ { 1 } ~ > ~ \frac { \gamma } { \alpha _ { H } }$ , the matrix $( I _ { N } \otimes S ) \ : - \ : \mu _ { 2 } ( H \otimes I _ { q } )$ is still Hurwitz and $\color{red} \widetilde { S } _ { d } ( t ) ( 1 _ { N } \otimes v ( t ) )$ <font color="red">will still decay to zero exponentially</font>. 
Thus, by Lemma 1, $\begin{array} { r } { \operatorname* { l i m } _ { t \to \infty } \tilde { \eta } _ { i } ( t ) ~ = ~ 0 } \end{array}$ exponentially and the proof is completed. 

原文中的$\mu_1$的条件是为了保证$\color{red} \widetilde { S } _ { d } ( t ) ( 1 _ { N } \otimes v ( t ) )$ 指数收敛。


现在对交叉项 $\tilde{S}_d(t)(1_N \otimes v(t))$ 取范数，并用次乘性 $\|AB\| \le \|A\|\cdot\|B\|$：

$$
\begin{aligned}
\|\tilde{S}_d(t)(1_N \otimes v(t))\|
&\le \|\tilde{S}_d(t)\| \cdot \|1_N \otimes v(t)\| \\[2mm]
&\le \bigl(C \, e^{-\mu_1 \alpha_H t}\bigr)
   \cdot \bigl(c' \, e^{\gamma t} \, t^{\,m-1}\bigr)
   && \text{代入 (S-d-decay) 与 (v-bound)} \\[2mm]
&= C c' \cdot t^{\,m-1} \cdot e^{-(\mu_1 \alpha_H - \gamma)\, t}. \tag{cross-bound}
\end{aligned}
$$

要保证指数收敛需要$\mu_1 \alpha_H - \gamma>0$。

- 指数部分的净指数为 $-(\mu_1 \alpha_H - \gamma) t$；
- 多项式部分 $t^{m-1}$ 只来自 $S$ 的非半单纯性，且**完全不依赖** $\mu_1, \alpha_H, \gamma$。