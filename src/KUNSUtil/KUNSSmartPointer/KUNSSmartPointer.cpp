/*
#include "KUNSSmartPointer.h"

template<class T>
KUNSSmartPointer<T>::KUNSSmartPointer() : m_pkPoint(NULL), m_pkCnt(NULL)
{

}

template<class T>
KUNSSmartPointer<T>::KUNSSmartPointer(T *pkPoint)
	: m_pkPoint(pkPoint), m_pkCnt(NULL)
{
	if( m_pkPoint )
		Inc();
}

template<class T>
KUNSSmartPointer<T>::KUNSSmartPointer(const KUNSSmartPointer &spPoint)
	: m_pkPoint(NULL), m_pkCnt(NULL)
{
	m_pkCnt = spPoint.m_pkCnt;
	m_pkPoint = spPoint.m_pkPoint;
	if( m_pkPoint )
		Inc();
}

template<class T>
KUNSSmartPointer<T>::~KUNSSmartPointer()
{
	Dec();
}

template<class T>
KUNSSmartPointer<T>::operator T*() const
{
	return m_pkPoint;
}

template<class T>
T& KUNSSmartPointer<T>::operator*() const
{
	return *m_pkPoint;
}

template<class T>
T* KUNSSmartPointer<T>::operator->() const
{
	return m_pkPoint;
}

template<class T>
KUNSSmartPointer<T>& KUNSSmartPointer<T>::operator=(const KUNSSmartPointer<T> &spPoint)
{
	if( m_pkPoint != spPoint.m_pkPoint )
	{
		if( m_pkPoint )
			Dec();
		m_pkPoint = spPoint.m_pkPoint;
		m_pkCnt = spPoint.m_pkCnt;
		if( m_pkPoint )
			Inc();
	}

	return *this;
}

template<class T>
KUNSSmartPointer<T>& KUNSSmartPointer<T>::operator=(T* pkPoint)
{
	if( m_pkPoint != pkPoint )
	{
		if( m_pkPoint )
			Dec();
		m_pkPoint = pkPoint;
		m_pkCnt = 0;
		if( m_pkPoint )
			Inc();
	}

	return *this;
}

template<class T>
bool KUNSSmartPointer<T>::operator== (T* pkPoint) const
{
	return m_pkPoint == pkPoint;
}

template<class T>
bool KUNSSmartPointer<T>::operator!= (T* pkPoint) const
{
	return m_pkPoint != pkPoint;
}

template<class T>
bool KUNSSmartPointer<T>::operator== (const KUNSSmartPointer &spPoint) const
{
	return m_pkPoint == spPoint.m_pkPoint;
}

template<class T>
bool KUNSSmartPointer<T>::operator!= (const KUNSSmartPointer &spPoint) const
{
	return m_pkPoint != spPoint.m_pkPoint;
}

template<class T>
int KUNSSmartPointer<T>::GetRefCount()
{ 
	return m_pkCnt?*m_pkCnt:0;
}	

template<class T>
bool KUNSSmartPointer<T>::IsNull()
{
	return m_pkPoint == NULL;
}

template<class T>
void KUNSSmartPointer<T>::Inc()
{
	if( m_pkCnt )
		++(*m_pkCnt);
	else
		m_pkCnt = new int(1);
}

template<class T>
void KUNSSmartPointer<T>::Dec()
{
	if( m_pkCnt )
	{
		if( --(*m_pkCnt) <= 0 )
		{
			//	해제 되는걸 확인하기 위해 printf추가
			printf("해제 : %d\n", *m_pkPoint);
			delete m_pkCnt;
			m_pkCnt = 0;
			delete m_pkPoint;
			m_pkPoint = 0;
		}
	}
}

*/